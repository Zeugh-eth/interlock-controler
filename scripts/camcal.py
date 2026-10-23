"""Gripper-camera self-calibration (hand-eye) from views of ONE stationary object.

Model: pixel = pinhole(f, principal point = image centre)( inverse(T_base_tool(q) @ X) @ P )
  X  camera pose in the tool frame (rotation vector + translation)  <- unknown, 6
  f  focal length in pixels                                         <- unknown, 1
  P  the object's 3D point in the base frame                        <- unknown, 3
Each frame (joints q from the arm, object centroid from the detector) gives 2 equations.
Fit by robust least squares with several starts (camera roll about the tool axis is unknown).

  fit(frames) -> dict(X=4x4, f, P, rms_px, n)
  project(cal, q, P) -> pixel ;  ray(cal, q, px) -> (origin, direction) in base frame
  locate_on_plane(cal, q, px, z) -> base-frame point where the pixel's ray meets height z
"""
import json, math
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
import kin

W, H = 640, 480
CAL_FILE = Path(__file__).resolve().parents[1] / "lab/arm-vision/gripper-camera-cal.json"


def _T(rvec, t):
    T = np.eye(4); T[:3, :3] = Rot.from_rotvec(rvec).as_matrix(); T[:3, 3] = t
    return T


def tool(q):
    return kin.tool_pose({j: q[j] for j in kin.CHAIN})


def _proj(Tbt, X, f, P):
    Tbc = Tbt @ X
    pc = Tbc[:3, :3].T @ (P - Tbc[:3, 3])
    if pc[2] <= 1e-4:
        return np.array([1e4, 1e4])
    return np.array([f * pc[0] / pc[2] + W / 2, f * pc[1] / pc[2] + H / 2])


def fit(frames, P0):
    Tbts = [tool(fr["q"]) for fr in frames]
    obs = np.array([[fr["cx"], fr["cy"]] for fr in frames])

    def resid(x):
        X = _T(x[0:3], x[3:6]); f = x[6]; P = x[7:10]
        return np.concatenate([_proj(T, X, f, P) - o for T, o in zip(Tbts, obs)])

    best = None
    for yaw in np.radians([0, 90, 180, 270]):
        for flip in (False, True):
            r0 = Rot.from_euler("z", yaw)
            if flip:
                r0 = r0 * Rot.from_euler("x", math.pi)
            for t0 in ([0.0, 0.03, -0.05], [0.03, 0.0, -0.05], [0.0, -0.03, -0.05], [-0.03, 0.0, -0.05]):
                p0 = np.array([P0[0], P0[1], min(max(P0[2], 0.01), 0.19)])
                x0 = np.r_[r0.as_rotvec(), t0, 500.0, p0]
                try:
                    r = least_squares(resid, x0, loss="soft_l1", f_scale=15.0, max_nfev=4000,
                                      # physical bounds: camera behind the jaw tips (tool z points out of
                                      # the gripper), object above the table plane z=0
                                      bounds=(np.r_[[-np.inf] * 3, -0.12, -0.12, -0.15, 250, P0[0] - 0.12, P0[1] - 0.12, 0.005],
                                              np.r_[[np.inf] * 3, 0.12, 0.12, 0.0, 1200, P0[0] + 0.12, P0[1] + 0.12, 0.20]))
                except ValueError:
                    continue
                rms = float(np.sqrt(np.mean(r.fun ** 2)))
                if best is None or rms < best[0]:
                    best = (rms, r.x)
    rms, x = best
    return {"X": _T(x[0:3], x[3:6]).tolist(), "f": float(x[6]), "P": x[7:10].tolist(), "rms_px": rms, "n": len(frames)}


def project(cal, q, P):
    return _proj(tool(q), np.array(cal["X"]), cal["f"], np.asarray(P))


def ray(cal, q, px):
    Tbc = tool(q) @ np.array(cal["X"])
    d_cam = np.array([(px[0] - W / 2) / cal["f"], (px[1] - H / 2) / cal["f"], 1.0])
    d = Tbc[:3, :3] @ d_cam
    return Tbc[:3, 3], d / np.linalg.norm(d)


def locate_on_plane(cal, q, px, z):
    o, d = ray(cal, q, px)
    s = (z - o[2]) / d[2]
    return o + s * d


def save(cal):
    CAL_FILE.write_text(json.dumps(cal, indent=1))


def load():
    return json.loads(CAL_FILE.read_text())
