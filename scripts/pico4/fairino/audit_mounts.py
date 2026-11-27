"""Offline comparison of the full-body URDF and the selected vendor ART7 model.

No ROS, SSH, PICO receiver or motor-control API is started by this script.
"""
import json
import math
import xml.etree.ElementTree as ET
import numpy as np

from paths import ROOT
from mounts import (MODEL_PATH, SOURCE_ARCHIVE, SOURCE_SHA256, MAPPING_ID,
                    BODY_FROM_BASE, BODY_FROM_DH, CHEST_FROM_DH, BASE_FROM_TELEOP,
                    tcp_in_chest)
from tcp_core import BOUNDS_DEG, MODEL_ID, pose_error
from vendor_kinematics import VendorKinematics
from lerobot.utils.rotation import Rotation


class FullModel:
    """Independent joint-by-joint URDF FK, starting at body_Link."""
    def __init__(self):
        self.joints = {j.get("name"): j for j in ET.parse(MODEL_PATH).getroot().findall("joint")}

    def origin(self, joint):
        o = joint.find("origin")
        roll, pitch, yaw = map(float, o.get("rpy").split())
        t = np.eye(4)
        t[:3, :3] = (Rotation.from_rotvec([0., 0., yaw]).as_matrix()
                     @ Rotation.from_rotvec([0., pitch, 0.]).as_matrix()
                     @ Rotation.from_rotvec([roll, 0., 0.]).as_matrix())
        t[:3, 3] = np.fromstring(o.get("xyz"), sep=" ")*1000.
        return t

    def flange_in_body(self, side, q):
        t = self.origin(self.joints[f"{side}_arm_fixed_j1"])
        for i, angle in enumerate(q, 1 if side == "left" else 8):
            j = self.joints[f"j{i}"]
            axis = np.fromstring(j.find("axis").get("xyz"), sep=" ")
            axis /= np.linalg.norm(axis)
            rot = np.eye(4)
            rot[:3, :3] = Rotation.from_rotvec(axis*angle).as_matrix()
            t = t @ self.origin(j) @ rot
        flange = np.eye(4)
        flange[0, 3] = -106.
        return t @ flange


def compare_models(samples=200):
    model = FullModel()
    kin = VendorKinematics()
    rng = np.random.default_rng(20261008)
    results = {}
    for side in ("left", "right"):
        errors = []
        for _ in range(samples):
            q = [math.radians(rng.uniform(lo+5, hi-5)) for lo, hi in BOUNDS_DEG]
            full = model.flange_in_body(side, q)
            vendor = BODY_FROM_DH[side] @ np.array(kin.forward_tcp(q)[0])
            errors.append(pose_error(full, vendor))
        results[side] = {"samples": samples, "max_position_error_mm": max(e[0] for e in errors),
                         "max_orientation_error_deg": max(e[1] for e in errors)}
    return results


def main():
    report = {
        "mapping": MAPPING_ID, "vendor_model": MODEL_ID,
        "source_archive": SOURCE_ARCHIVE, "urdf_sha256": SOURCE_SHA256,
        "frame": "body_Link origin, X forward (+body Y), Y left (-body X), Z up (+body Z); not a ground frame",
        "body_from_urdf_arm_base_mm": {s: t.tolist() for s, t in BODY_FROM_BASE.items()},
        "body_from_vendor_dh_mm": {s: t.tolist() for s, t in BODY_FROM_DH.items()},
        "chest_from_vendor_dh_mm": {s: t.tolist() for s, t in CHEST_FROM_DH.items()},
        "base_from_teleop": {s: t.tolist() for s, t in BASE_FROM_TELEOP.items()},
        "model_comparison": compare_models(),
        "unresolved": [
            "Only the sum of the three folding-joint angles has an upright calibration.",
            "The RD53 j20 CAD lower limit is -1.1868 rad; check measured hardware positions separately.",
            "CAD nominal values are not physical calibration; actual hardware subtype/nameplate not verified online.",
        ],
        "robot_motion_executed": False,
    }
    for result in report["model_comparison"].values():
        if result["max_position_error_mm"] >= .02 or result["max_orientation_error_deg"] >= .01:
            raise ValueError("整机 URDF 与 ART7 运动学不一致")
    logs = ROOT/".local/pico4/fairino-logs"
    sources = sorted(p for p in logs.glob("20*.jsonl") if not p.name.endswith(".actions.jsonl"))
    if sources:
        hw = None
        for line in sources[-1].open():
            row = json.loads(line)
            if isinstance(row.get("hardware"), dict):
                hw = row["hardware"]
        if hw:
            report["recorded_pose_source"] = sources[-1].name
            report["recorded_flange_in_chest_mm"] = {
                s: tcp_in_chest(s, VendorKinematics().forward_tcp(hw["position"][a:a+7])[0]).tolist()
                for s, a in (("left", 0), ("right", 7))}
    logs.mkdir(exist_ok=True, parents=True)
    output = logs/"validation-urdf-mounts.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps(report["model_comparison"], ensure_ascii=False, indent=2))
    print(output)


if __name__ == "__main__":
    main()
