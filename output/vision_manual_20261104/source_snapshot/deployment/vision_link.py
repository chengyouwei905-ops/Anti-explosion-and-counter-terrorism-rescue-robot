"""
vision_link.py —— K230 端串口传输与命令接收（后台线程版）

协议格式（与 STM32 工程 Protocol/Inc/proto_vision.h 对齐）:

    +------+------+-----+-----+-----+---------------+------+
    | 0xAA | 0x55 | LEN | SEQ | CMD |    PAYLOAD    | CRC8 |
    +------+------+-----+-----+-----+---------------+------+

    LEN  : PAYLOAD 字节数 (0 ~ 64)
    SEQ  : 帧序号（STM32 端自增，用于丢帧检测）
    CMD  : 命令字（原接口仍可用，新增任务业务分配见 protocol.py）
    CRC8 : 对 LEN/SEQ/CMD/PAYLOAD 计算
           CRC-8/MAXIM（poly 0x8C 反射、init 0x00），与 STM32 端 proto_vision_crc8 一致

用法（主线程只需要一个接口 vision.get()）:

    from vision_link import VisionLink
    vision = VisionLink()                 # 后台线程自动开始收数
    ...
    frame = vision.get()                  # 最近一帧：Frame(seq, cmd, payload, ts)，无数据返回 None
    if frame:
        # frame.seq / frame.cmd / frame.payload(bytes) / frame.ts
        ...
    vision.deinit()                       # 退出前停止线程并释放 UART

说明:
- 后台线程负责接收、解析、CRC 和重同步；主线程取命令并发送结果。
- 默认保留最新帧接口；get_new() 按本地接收编号判断，不受 SEQ 回绕影响。
- queue_size>0 时另启有界命令队列；STOP 等 priority_cmds 优先取出。
- send(cmd, payload) 按同一格式回发一帧，并检查 UART 写入长度。
"""

import time
import _thread

SOF1 = 0xAA
SOF2 = 0x55
MAX_PAYLOAD = 64
OVERHEAD = 6
_HEADER = b"\xaa\x55"


def _ticks_ms():
    method = getattr(time, "ticks_ms", None)
    return method() if method is not None else int(time.monotonic()*1000)


def _ticks_diff(a, b):
    method = getattr(time, "ticks_diff", None)
    return method(a, b) if method is not None else a-b


def _sleep_ms(value):
    method = getattr(time, "sleep_ms", None)
    if method is not None:
        method(value)
    else:
        time.sleep(value/1000.0)


def crc8(data, crc=0):
    """CRC-8/MAXIM（反射多项式 0x8C，初值 0x00），与 STM32 端 proto_vision_crc8 完全一致。"""
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc & 0xFF


class Frame:
    """解出的一帧。payload 为 bytes。"""
    __slots__ = ("seq", "cmd", "payload", "ts", "rx_id")

    def __init__(self, seq, cmd, payload, ts, rx_id=0):
        self.seq = seq
        self.cmd = cmd
        self.payload = payload
        self.ts = ts
        self.rx_id = rx_id

    def __repr__(self):
        return "Frame(seq=%d, cmd=0x%02X, payload=%s)" % (self.seq, self.cmd, self.payload.hex())


class VisionLink:
    def __init__(self, uart_id=None, baudrate=115200, tx_pin=11, rx_pin=12,
                 queue_size=0, priority_cmds=(), read_chunk=128, uart=None, start_thread=True):
        if type(queue_size) is not int or not 0 <= queue_size <= 64:
            raise ValueError("INVALID_COMMAND_QUEUE_SIZE")
        if type(read_chunk) is not int or not 1 <= read_chunk <= 512:
            raise ValueError("INVALID_UART_READ_CHUNK")
        if uart is None:
            from machine import UART, FPIOA
            if uart_id is None:
                uart_id = UART.UART2
            if uart_id != UART.UART2:
                raise ValueError("THIS_PIN_CONFIGURATION_REQUIRES_UART2")
            fpioa = FPIOA()
            fpioa.set_function(tx_pin, FPIOA.UART2_TXD)
            fpioa.set_function(rx_pin, FPIOA.UART2_RXD)
            uart = UART(uart_id, baudrate=baudrate, bits=UART.EIGHTBITS,
                        parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        self.uart = uart
        self._lock = _thread.allocate_lock()
        self._rx_io_lock = _thread.allocate_lock()
        self._accept_commands = True
        self._queue, self._priority, self._overflow = [], None, None
        self._queue_size, self._priority_cmds, self._read_chunk = queue_size, tuple(priority_cmds), read_chunk
        self._rx_id, self._last_rx_id = 0, -1
        self.queue_overflows = self.commands_discarded = 0
        self._rx_exited, self._closed, self._rx_error = not start_thread, False, None

        self._buf = bytearray()        # 收包缓冲（保存跨帧残留）
        self._latest = None            # 最近一帧 Frame
        self._last_seq = -1            # get_new() 用
        self.rx_bytes = 0              # 统计：累计收到的字节数
        self.ok_frames = 0             # 统计：CRC 通过的帧数
        self.crc_errors = 0            # 统计：CRC 错误次数
        self.len_errors = 0            # 统计：长度非法次数
        self.tx_seq = 0                # 发送帧序号
        self._running = True
        self._tid = None
        if start_thread:
            try:
                self._tid = _thread.start_new_thread(self._rx_task, ())
            except BaseException:
                self._running = False
                self._rx_exited = True
                self.uart.deinit()
                self._closed = True
                raise

    # ------------------------------------------------------------------
    # 主线程接口
    # ------------------------------------------------------------------
    def get(self):
        """返回最近一帧 Frame，没有收到过数据时返回 None。"""
        return self._latest

    def get_new(self):
        """返回"新"一帧：与上次取到的是同一帧时返回 None。"""
        self._lock.acquire()
        try:
            f = self._latest
            if f is None or f.rx_id == self._last_rx_id:
                return None
            self._last_rx_id = f.rx_id
            return f
        finally:
            self._lock.release()

    def pop_command(self):
        self._lock.acquire()
        try:
            if self._priority is not None:
                frame, self._priority = self._priority, None
                self.commands_discarded += len(self._queue)
                self._queue = []
                self._overflow = None
                return frame
            return self._queue.pop(0) if self._queue else None
        finally:
            self._lock.release()

    def pop_priority(self):
        self._lock.acquire()
        try:
            if self._priority is None:
                return None
            frame, self._priority = self._priority, None
            self.commands_discarded += len(self._queue)
            self._queue = []
            self._overflow = None
            return frame
        finally:
            self._lock.release()

    def take_overflow(self):
        self._lock.acquire()
        try:
            frame, self._overflow = self._overflow, None
            if frame is not None:
                self.commands_discarded += len(self._queue)
                self._queue = []
            return frame
        finally:
            self._lock.release()

    def healthy(self):
        return self._running and not self._closed and self._rx_error is None

    def diagnostics(self):
        return {"healthy": self.healthy(), "rx_error": self._rx_error,
                "queue_depth": len(self._queue), "queue_overflows": self.queue_overflows,
                "commands_discarded": self.commands_discarded, "rx_thread_exited": self._rx_exited}

    def stats(self):
        """(rx_bytes, ok_frames, crc_errors, len_errors)"""
        return (self.rx_bytes, self.ok_frames, self.crc_errors, self.len_errors)

    def pack(self, cmd, payload=b""):
        """按协议组一帧（含 CRC），返回 bytes，不发送。payload 必须为 bytes。"""
        if len(payload) > MAX_PAYLOAD:
            raise ValueError("payload too long")
        if type(cmd) is not int or not 0 <= cmd <= 255:
            raise ValueError("invalid cmd")
        head = bytes((len(payload), self.tx_seq & 0xFF, cmd))
        self.tx_seq = (self.tx_seq + 1) & 0xFF
        body = head + bytes(payload)
        return _HEADER + body + bytes((crc8(body),))

    def send(self, cmd, payload=b""):
        """按同一格式回发一帧给 STM32。"""
        if self._closed:
            raise OSError("UART_CLOSED")
        frame = self.pack(cmd, payload)
        written = self.uart.write(frame)
        if written != len(frame):
            raise OSError("UART_SHORT_WRITE")
        return frame

    def send_raw(self, data):
        """Main-thread lifecycle token: no AA55 wrapper, CRC or newline."""
        if self._closed: raise OSError("UART_CLOSED")
        data=bytes(data)
        if self.uart.write(data)!=len(data): raise OSError("UART_SHORT_WRITE")
        return data

    def set_command_acceptance(self, enabled):
        """Atomically purge stale/partial RX at a user-key lifecycle boundary."""
        self._rx_io_lock.acquire()
        try:
            self._lock.acquire()
            try:
                self._accept_commands=False
                self.commands_discarded+=len(self._queue)+(1 if self._priority is not None else 0)
                self._queue,self._priority,self._overflow=[],None,None
                self._buf=bytearray()
                self._latest=None
                # Reader and this purge cannot consume the UART simultaneously.
                # Bounded drain: never hang on a continuously transmitting peer.
                for _ in range(32):
                    available=self.uart.any()
                    if not available: break
                    self.uart.read(min(available,512))
                else:
                    if self.uart.any(): raise OSError("UART_RX_NOT_QUIET")
                self._accept_commands=bool(enabled)
            finally: self._lock.release()
        finally: self._rx_io_lock.release()

    def deinit(self, wait_ms=300):
        """停止接收线程并释放串口，退出前调用。"""
        if self._closed:
            return True
        self._running = False
        started = _ticks_ms()
        while not self._rx_exited and _ticks_diff(_ticks_ms(), started) < wait_ms:
            _sleep_ms(5)
        if not self._rx_exited:
            self._rx_error = "UART_SHUTDOWN_TIMEOUT"
            return False
        self.uart.deinit()
        self._closed = True
        return True

    # ------------------------------------------------------------------
    # 后台接收线程
    # ------------------------------------------------------------------
    def _rx_task(self):
        try:
            while self._running:
                data=None
                self._rx_io_lock.acquire()
                try:
                    available = self.uart.any()
                    if available:
                        data = self.uart.read(min(available, self._read_chunk))
                        if data: self._feed(data)
                finally: self._rx_io_lock.release()
                if not data: _sleep_ms(5)
        except SystemExit:
            if self._running:
                self._rx_error = "RX_THREAD_EXITED"
        except BaseException as e:
            self._rx_error = type(e).__name__ + ":" + str(e)
        finally:
            self._running = False
            self._rx_exited = True

    def _publish(self, seq, cmd, payload):
        self._lock.acquire()
        try:
            if self._accept_commands: self._publish_locked(seq,cmd,payload)
        finally:
            self._lock.release()

    def _publish_locked(self,seq,cmd,payload):
        self._rx_id += 1
        frame = Frame(seq, cmd, payload, _ticks_ms(), self._rx_id)
        self._latest = frame
        if self._queue_size:
            if cmd in self._priority_cmds: self._priority = frame
            elif len(self._queue) < self._queue_size: self._queue.append(frame)
            else:
                self.queue_overflows += 1
                self._overflow = frame

    def _feed(self, data):
        """解析字节流（支持半帧/粘包/垃圾数据），线程内调用。"""
        self._lock.acquire()
        try:
            if not self._accept_commands:
                self.rx_bytes+=len(data)
                self._buf=bytearray()
                return
            self._feed_locked(data)
        finally: self._lock.release()

    def _feed_locked(self,data):
        self.rx_bytes += len(data)
        buf = self._buf + data
        while True:
            i = buf.find(_HEADER)
            if i < 0:
                if len(buf) > 1:
                    buf = buf[-1:]     # 只保留最后一个字节（可能是半个帧头）
                break
            if i:
                buf = buf[i:]          # 丢掉帧头之前的垃圾
            if len(buf) < OVERHEAD:
                break                  # 帧还没收全
            n = buf[2]
            if n > MAX_PAYLOAD:
                self.len_errors += 1
                buf = buf[1:]          # 长度非法，跳过 1 字节重新同步
                continue
            total = OVERHEAD + n
            if len(buf) < total:
                break                  # 帧还没收全
            if crc8(buf[2:total - 1]) == buf[total - 1]:
                self.ok_frames += 1
                self._publish_locked(buf[3], buf[4], bytes(buf[5:total - 1]))
                buf = buf[total:]
            else:
                self.crc_errors += 1
                buf = buf[1:]          # CRC 错，跳过 1 字节重新同步
        self._buf = buf
