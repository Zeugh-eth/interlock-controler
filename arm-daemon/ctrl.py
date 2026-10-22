"""Control token and STOP state for arm-daemon. Pure: no hardware, no network.

holder is "human" or "agent". Only a request that proves it is human (correct key) may change it.
STOP can be set by anyone (or by the arm-agent/STOP file); only a human may resume.
"""
import hmac
import time
from pathlib import Path

HOLDERS = ("human", "agent", "leader", "released")   # leader = teleop through the daemon; released = bus freed


class ControlState:
    def __init__(self, key: str, stop_file: Path):
        self._key = key
        self._stop_file = Path(stop_file)
        self.holder = "human"
        self.since = time.time()
        self._stop = {"set": False, "reason": "", "by": ""}

    # --- identity -------------------------------------------------------------------------------
    def is_human(self, header_key) -> bool:
        if not header_key:
            return False
        try:
            return hmac.compare_digest(str(header_key).encode("utf-8"), self._key.encode("utf-8"))
        except Exception:
            return False

    # --- holder ---------------------------------------------------------------------------------
    def set_holder(self, holder: str, human: bool):
        if holder not in HOLDERS:
            return False, "unknown holder"
        if not human:
            return False, "human key required"
        if holder != self.holder:
            self.holder = holder
            self.since = time.time()
        return True, ""

    # --- stop -----------------------------------------------------------------------------------
    def stop(self, reason: str, by: str) -> None:
        self._stop = {"set": True, "reason": reason, "by": by}

    def resume(self, human: bool):
        if not human:
            return False, "human key required"
        self._stop = {"set": False, "reason": "", "by": ""}
        try:
            self._stop_file.unlink()
        except FileNotFoundError:
            pass
        return True, ""

    def stop_state(self) -> dict:
        if self._stop["set"]:
            return dict(self._stop)
        if self._stop_file.exists():
            return {"set": True, "reason": f"file {self._stop_file}", "by": "file"}
        return {"set": False, "reason": "", "by": ""}

    # --- write gate -----------------------------------------------------------------------------
    def can_write(self, human: bool):
        if self.stop_state()["set"]:
            return False, "stop is set"
        if self.holder == "released":
            return False, "bus released"
        if self.holder == "leader":
            return False, "leader has control"
        if self.holder == "human" and not human:
            return False, "human has control"
        return True, ""
