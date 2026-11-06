"""User key gates all vision work. Lifecycle text is carried inside the same AA55/CRC8 packet."""
import runtime_config as cfg
import protocol as p


class RunControl:
    def __init__(self,link,manager,button):
        self.link,self.manager,self.button=link,manager,button
        self.running=False
        self.generation=0
        self.error=None
        self.starts=self.stops=0
        self.manager.pause()
        self.link.set_command_acceptance(False)

    @property
    def state(self):
        return "FAULT" if self.error else "STARTED" if self.running else "WAIT_START"

    def _start(self):
        # Purge before START; an immediate MCU reply after write must survive.
        self.link.set_command_acceptance(True)
        try: self.link.send(p.STATUS,cfg.TOTAL_START_BYTES)
        except BaseException:
            self.link.set_command_acceptance(False)
            raise
        self.manager.resume()
        self.running=True
        self.starts+=1
        self.generation+=1
        print("K230_TOTAL_START",cfg.TOTAL_START_BYTES.decode())

    def _stop(self):
        self.running=False
        self.generation+=1
        # Pause/discard poses before writing STOP; no binary CANCELLED packet.
        try:
            self.manager.pause()
            if cfg.TOTAL_STOP_POLICY=="restart":self.manager.reset_session()
        finally:
            try:
                self.link.send(p.STATUS,cfg.TOTAL_STOP_BYTES)
                self.stops+=1
                print("K230_TOTAL_STOP",cfg.TOTAL_STOP_BYTES.decode())
            finally: self.link.set_command_acceptance(False)

    def poll(self):
        if self.error: return
        try:
            if not self.button.healthy(): raise OSError("USER_BUTTON_FAILED:"+str(self.button.error))
            for _ in range(8):
                if not self.button.pressed(): break
                if self.running: self._stop()
                else: self._start()
        except Exception as exc:
            self.error=type(exc).__name__+":"+str(exc)
            if self.running:
                try: self._stop()
                except Exception: pass
            self.running=False
            self.manager.pause()
            try: self.link.set_command_acceptance(False)
            except Exception: pass
            print("K230_CONTROL_ERROR",self.error)

    def close(self):
        if self.running:
            try: self._stop()
            except Exception as exc:
                self.error=type(exc).__name__+":"+str(exc)
                print("K230_STOP_TX_FAILED",self.error)
