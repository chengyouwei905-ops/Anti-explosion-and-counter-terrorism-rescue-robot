"""Debounced GPIO53 events. Worker only reads GPIO; main owns UART and tasks."""
import _thread
import runtime_config as cfg
from camera_service import ticks_ms,ticks_diff
from vision_link import _sleep_ms


class DebouncedKey:
    def __init__(self,pin,active=1,debounce_ms=40,clock=ticks_ms,diff=ticks_diff):
        self.pin,self.active,self.debounce_ms=pin,active,debounce_ms
        self.clock,self.diff=clock,diff
        self.raw=self.stable=pin.value()
        self.changed_at=clock()
        self.armed=self.stable!=active

    def pressed(self):
        now,value=self.clock(),self.pin.value()
        if value!=self.raw: self.raw,self.changed_at=value,now
        if self.raw!=self.stable and self.diff(now,self.changed_at)>=self.debounce_ms:
            self.stable=self.raw
            if self.stable!=self.active: self.armed=True
            elif self.armed:
                self.armed=False
                return True
        return False


class UserButton:
    def __init__(self,pin=None,start_thread=True,clock=ticks_ms,diff=ticks_diff):
        if pin is None:
            from machine import FPIOA,Pin
            mapper=FPIOA()
            mapper.set_function(cfg.USER_KEY_GPIO,FPIOA.GPIO0+cfg.USER_KEY_GPIO)
            pin=Pin(cfg.USER_KEY_GPIO,Pin.IN,Pin.PULL_DOWN if cfg.USER_KEY_ACTIVE else Pin.PULL_UP)
        self.key=DebouncedKey(pin,cfg.USER_KEY_ACTIVE,cfg.USER_KEY_DEBOUNCE_MS,clock,diff)
        self._lock=_thread.allocate_lock()
        self._pending=0
        self._running=True
        self._exited=not start_thread
        self.error=None
        if start_thread:
            try: _thread.start_new_thread(self._worker,())
            except BaseException:
                self._running=False
                self._exited=True
                raise

    def sample(self):
        if not self.key.pressed(): return
        self._lock.acquire()
        try:
            if self._pending>=8:
                self.error="BUTTON_EVENT_OVERFLOW"
                self._running=False
            else: self._pending+=1
        finally: self._lock.release()

    def _worker(self):
        try:
            while self._running:
                self.sample()
                _sleep_ms(cfg.USER_KEY_POLL_MS)
        except BaseException as exc:
            self.error=type(exc).__name__+":"+str(exc)
        finally:
            self._running=False
            self._exited=True

    def pressed(self):
        self._lock.acquire()
        try:
            if not self._pending: return False
            self._pending-=1
            return True
        finally: self._lock.release()

    def healthy(self): return self._running and self.error is None

    def close(self,wait_ms=300):
        self._running=False
        started=ticks_ms()
        while not self._exited and ticks_diff(ticks_ms(),started)<wait_ms: _sleep_ms(5)
        return self._exited
