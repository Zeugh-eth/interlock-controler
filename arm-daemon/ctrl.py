"""Control token and STOP state for arm-daemon. Pure: no hardware, no network.

holder is "human" or "agent". Only a request that proves it is human (correct key) may change it.
STOP can be set by anyone (or by the arm-agent/STOP file); only a human may resume.
Agent requests come from the LOCAL agent (on this machine, the default) or the REMOTE one (mcp_server
--http, header X-Arm-Agent: remote). While the holder is agent both may write, the local one first: a
remote write is refused for LOCAL_LEASE_S after the local agent last asked, and the human can block the
remote agent from the dashboard.
"""
import hmac
import time
from pathlib import Path

HOLDERS = ("human", "agent", "leader", "released")   # leader = teleop through the daemon; released = bus freed
AGENTS = ("local", "remote")
LOCAL_LEASE_S = 20.0


class ControlState:
    def __init__(self, key: str, stop_file: Path, auth_lock=None):
        self._key = key
        self._stop_file = Path(stop_file)
        self.auth_lock = auth_lock          # authlock.AuthLock: agent mode needs the device code once
        self.holder = "human"
        self.since = time.time()
        self._stop = {"set": False, "reason": "", "by": ""}
        self.remote_allowed = True
        self._agent_t = {a: 0.0 for a in AGENTS}      # last time each agent asked to write

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
        if holder == "agent" and self.auth_lock is not None and not self.auth_lock.unlocked:
            return False, "device auth required"
        if holder != self.holder:
            self.holder = holder
            self.since = time.time()
        return True, ""

    # --- local / remote agent ---------------------------------------------------------------------
    def set_remote_allowed(self, allowed: bool, human: bool):
        if not human:
            return False, "human key required"
        self.remote_allowed = bool(allowed)
        return True, ""

    def touch(self, agent: str) -> None:
        self._agent_t[agent if agent in AGENTS else "local"] = time.time()

    def local_active(self) -> bool:
        return time.time() - self._agent_t["local"] < LOCAL_LEASE_S

    def agents_state(self) -> dict:
        now = time.time()
        return {"remote_allowed": self.remote_allowed, "lease_s": LOCAL_LEASE_S,
                **{a: {"last_t": t or None, "active": bool(t) and now - t < LOCAL_LEASE_S} for a, t in self._agent_t.items()}}

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
    def can_write(self, human: bool, agent: str = "local"):
        if self.stop_state()["set"]:
            return False, "stop is set"
        if self.holder == "released":
            return False, "bus released"
        if self.holder == "leader":
            return False, "leader has control"
        if self.holder == "human" and not human:
            return False, "human has control"
        if not human and agent == "remote":
            if not self.remote_allowed:
                return False, "remote agent blocked on the dashboard"
            if self.local_active():
                return False, "local agent has control"
        return True, ""
