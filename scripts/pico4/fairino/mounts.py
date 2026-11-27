"""Manufacturer nominal arm mounts, relative to the upper body (not ground).

Lengths here are mm, matching the vendor FK/IK wrapper. The source URDF uses m.
Only the two fixed mounts and the DH/base-origin offsets are used at runtime;
the torso joint numbering, limits and inertias are not imported into control.
"""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

MODEL_PATH = Path(__file__).resolve().parent / "models/fairino_wheel_legged_robot_description.urdf"
SOURCE_ARCHIVE = "RD53-轮足机器人URDF和模型-v1.0-20261030"
SOURCE_SHA256 = "96d4b4fb6e6a584bc55e1e9f1cf538ff9b02cc2093f1cda1cc983cd48199be9c"
MAPPING_ID = "art7-both-chest-urdf-20261030-v1"

# body_Link has -X toward the left arm, +Z toward the head, hence +Y forward.
# Keep its origin; expose a right-handed chest frame with X forward/Y left/Z up.
CHEST_FROM_BODY = np.array([[0., 1., 0., 0.],
                            [-1., 0., 0., 0.],
                            [0., 0., 1., 0.],
                            [0., 0., 0., 1.]])


def origin_matrix(joint):
    """URDF parent<-joint transform: Rz(yaw) Ry(pitch) Rx(roll)."""
    origin = joint.find("origin")
    r, p, y = map(float, origin.get("rpy").split())
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                 [-sp, cp*sr, cp*cr]]
    t[:3, 3] = np.fromstring(origin.get("xyz"), sep=" ") * 1000.
    return t


def load_mounts():
    data = MODEL_PATH.read_bytes()
    if hashlib.sha256(data).hexdigest() != SOURCE_SHA256:
        raise ValueError("双臂安装 URDF 已变更，需要重新核对模型后才能使用")
    joints = {j.get("name"): j for j in ET.fromstring(data).findall("joint")}
    body_from_base, body_from_dh = {}, {}
    for side, first_name in (("left", "j1"), ("right", "j8")):
        mount = joints[f"{side}_arm_fixed_j1"]
        if (mount.get("type") != "fixed" or mount.find("parent").get("link") != "body_Link"
                or mount.find("child").get("link") != f"{side}_arm_base_Link"):
            raise ValueError(f"{side} 安装链与已核对的模型不符")
        body_from_base[side] = origin_matrix(mount)
        # The manufacturer DH starts at x=y=0, while the CAD arm base has a
        # small XY origin offset before j1/j8. d1=187 mm is already in DH.
        first = joints[first_name]
        xyz = np.fromstring(first.find("origin").get("xyz"), sep=" ") * 1000.
        if first.find("parent").get("link") != f"{side}_arm_base_Link" or abs(xyz[2]-187.) > 1e-6:
            raise ValueError(f"{side} 首关节与 ART7-R7 DH 不符")
        base_from_dh = np.eye(4)
        base_from_dh[:2, 3] = xyz[:2]
        body_from_dh[side] = body_from_base[side] @ base_from_dh
    return body_from_base, body_from_dh


BODY_FROM_BASE, BODY_FROM_DH = load_mounts()
CHEST_FROM_DH = {side: CHEST_FROM_BODY @ t for side, t in BODY_FROM_DH.items()}
BASE_FROM_TELEOP = {side: t[:3, :3].T.copy() for side, t in CHEST_FROM_DH.items()}


def tcp_in_chest(side, tcp_in_dh):
    """Convert a vendor flange pose to the common chest frame; no tool shift."""
    return CHEST_FROM_DH[side] @ np.asarray(tcp_in_dh, dtype=float)


def tcp_in_base(side, tcp_in_chest_frame):
    """Convert a common-chest flange goal back into the vendor IK base."""
    t = CHEST_FROM_DH[side]
    inverse = np.eye(4)
    inverse[:3, :3] = t[:3, :3].T
    inverse[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return inverse @ np.asarray(tcp_in_chest_frame, dtype=float)
