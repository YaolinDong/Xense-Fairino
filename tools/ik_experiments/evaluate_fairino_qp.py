#!/usr/bin/env python3
"""Evaluate the real native solver offline. Write all metrics before applying acceptance gates."""
from collections import Counter
import json
import math
import os
from pathlib import Path
import sys
from time import perf_counter_ns
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from apps.fairino_tcp_probe.tcp_core import FLANGE_OFFSET_MM, pose_error, tcp_forward_matrix
from apps.fairino_tcp_probe.tool_frames import for_arm, tool_config
from apps.workbench.model import RobotModel, rotation
from scripts.pico4.fairino.home_presets import HOME_PRESETS
from scripts.pico4.fairino.mounts import BODY_FROM_DH, CHEST_FROM_BODY
from tools.ik_experiments.fairino_qp_art7 import (
    ARM_LIMITS, CACHE, ORIENTATION_DEG, POSITION_MM, SIDES, TORSO_MAPPING,
    Art7FairinoQP, WORLD_FROM_BASE, model_xml, torso_angles, urdf_coordinates,
)

RNG_SEED = 20261119
RANDOM_PAIRS = 160
TORSOS = ((50, 0, 0, 0), (66.7, 20, 0, -25), (100, 4, 12, 35), (95, 0, -15, -40))


def body_pose(body, height, forward=0., roll=0., yaw=0.):
    return np.asarray(body.pose(dict(height=height, forward=forward, roll=roll, yaw=yaw), [0.] * 20))


def homes():
    result = {}
    for name, (_, path) in HOME_PRESETS.items():
        data = json.loads(path.read_text())['joint_degrees']
        result[name] = np.radians(data['left'] + data['right'])
    return result


def solver_reference():
    """Independent NumPy FK of exactly the URDF handed to the solver."""
    model = RobotModel()
    model.joints = ET.fromstring(model_xml()).findall('joint')
    return model


def world_frame(raw_m):
    pose = WORLD_FROM_BASE @ raw_m
    pose[:3, 3] *= 1000.
    return pose


def update_maximum(record, errors, case):
    record['count'] = record.get('count', 0) + 1
    for key, value in zip(('position_mm', 'orientation_deg'), errors):
        if key not in record or value > record[key]:
            record[key], record[key + '_case'] = value, case


def arm_samples():
    yield 'zero', np.zeros(14)
    yield from homes().items()
    # All axes together and each boundary independently expose small CAD/DH discrepancies.
    for col, tag in enumerate(('lower', 'upper')):
        boundary = ARM_LIMITS[:, col] + (1 if col == 0 else -1) * 1e-8
        yield tag, boundary
        for i, value in enumerate(boundary):
            q = np.zeros(14)
            q[i] = value
            yield f'{tag}_J{i + 1}', q
    random = np.random.default_rng(RNG_SEED)
    for i, q in enumerate(random.uniform(ARM_LIMITS[:, 0], ARM_LIMITS[:, 1], (RANDOM_PAIRS, 14))):
        yield f'random_{i}', q


def forward_audit(solvers, tools, model):
    arm, chain = {mode: {} for mode in tools}, {}
    for state in TORSOS:
        q = body_pose(model.body, *state)
        q[14] = .83  # Measured head deliberately differs from model zero.
        for label, q_arm in arm_samples():
            q[:14] = q_arm
            raw, _ = model.transforms(urdf_coordinates(q, solvers['flange'].upright_rad))
            body = world_frame(raw['body_Link'])
            native = solvers['flange'].frames(q)
            case = dict(torso=state, arms=label)
            for frame, matrix in raw.items():
                update_maximum(chain, pose_error(world_frame(matrix), native[frame]), dict(case, link=frame))
            for side, angles in zip(SIDES, q_arm.reshape(2, 7)):
                dh_flange = body @ BODY_FROM_DH[side] @ tcp_forward_matrix(angles.tolist())
                wrist = world_frame(raw[side + '_waist2_Link'])
                flange = np.eye(4)
                flange[:3, 3] = FLANGE_OFFSET_MM
                for mode, solver in solvers.items():
                    tcp = solver.forward(side, q)
                    tool = np.asarray(for_arm(tools[mode], side))
                    err = pose_error(tcp, dh_flange @ tool)
                    record = arm[mode]
                    record['outside_tolerance'] = record.get('outside_tolerance', 0) + int(
                        err[0] > POSITION_MM or err[1] > ORIENTATION_DEG)
                    if err[0] > record.get('position_mm', -1):
                        # dTCP = dFlange + (R_URDF - R_DH) * tool_offset.
                        offset = tool[:3, 3]
                        rotation_term = (tcp[:3, :3] - dh_flange[:3, :3]) @ offset
                        translation_term = (tcp[:3, 3] - tcp[:3, :3] @ offset) - dh_flange[:3, 3]
                        record['worst_position_breakdown'] = dict(
                            arm_rad=angles.tolist(), flange_to_tip_length_mm=float(np.linalg.norm(offset)),
                            flange_error_vector_mm=translation_term.tolist(),
                            rotated_offset_error_vector_mm=rotation_term.tolist(),
                            rotated_offset_error_mm=float(np.linalg.norm(rotation_term)))
                    update_maximum(record, err, dict(case, side=side))
                    update_maximum(chain, pose_error(tcp, wrist @ flange @ tool),
                                   dict(case, side=side, tool=mode))
    return arm, chain


def expected_rotation(forward, roll, yaw):
    # Body controls define pitch and roll before waist yaw.
    return (rotation((0, 1, 0), math.radians(forward))
            @ rotation((1, 0, 0), math.radians(roll))
            @ rotation((0, 0, 1), math.radians(yaw)))


def mapping_audit(solver, model):
    body = model.body
    low, _ = model.transforms(urdf_coordinates(body_pose(body, 0), solver.upright_rad))
    reference = world_frame(low['hip_Link'])[:3, 3]
    observations = []
    for state in (*TORSOS, (100, 0, 0, 0), (100, 0, 0, 10), (100, 0, 0, -10),
                  (100, 0, 10, 0), (100, 0, -10, 0), (50, 25, 0, 0)):
        height, forward, roll, yaw = state
        q = body_pose(body, *state)
        frames = solver.frames(q)
        chest = frames['body_Link'] @ CHEST_FROM_BODY.T
        expected = np.eye(4)
        expected[:3, :3] = expected_rotation(forward, roll, yaw)
        _, angle = pose_error(chest, expected)
        delta = frames['hip_Link'][:3, 3] - reference
        observations.append(dict(state=state, chest_orientation_error_deg=angle,
                                 chest_forward=chest[:3, 0].tolist(), chest_up=chest[:3, 2].tolist(),
                                 pivot_height_mm=delta[2], expected_height_mm=height * body.travel / 100,
                                 pivot_horizontal_drift_mm=float(np.linalg.norm(delta[:2]))))
    return dict(upright_joint_sum_deg=body.neutral, observations=observations,
                max_orientation_error_deg=max(o['chest_orientation_error_deg'] for o in observations),
                max_height_error_mm=max(abs(o['pivot_height_mm'] - o['expected_height_mm'])
                                        for o in observations),
                max_horizontal_drift_mm=max(o['pivot_horizontal_drift_mm'] for o in observations))


def height_limits(solver, body):
    # Each fold angle is monotonic along this calibrated lift path.
    rows = []
    for forward in (0, 5, 15, 25, 35, 45):
        cutoffs = {}
        for i, (_, name, _) in enumerate(TORSO_MAPPING):
            lo, hi = solver.torso_limits[i]
            def angle(height):
                return torso_angles(body_pose(body, height, forward), solver.upright_rad)[i]
            if angle(100) < lo or angle(100) > hi:
                raise ValueError(f'Recheck height interval topology for {name}')
            if lo <= angle(0) <= hi:
                continue
            left, right = 0., 100.
            for _ in range(52):
                mid = (left + right) / 2
                if not lo <= angle(mid) <= hi:
                    left = mid
                else:
                    right = mid
            cutoffs[name] = right
        rows.append(dict(forward_deg=forward, rejected_from_zero_to_percent_exclusive=cutoffs))
    return rows


# Reproduce the *initial* upstream box samples; a failed hold solve may move
# away from its seed and reports the returned candidate's clearance instead.
BOXES_MM = {'body_Link': ((0, -7.52, 225.5), (160.01, 168.69, 224.5)),
            'waist_Link': ((30.04, 0, -49.09), (109.97, 118, 185.91)),
            'base_link': ((0, -149.33, 20.25), (301.65, 382.33, 191.25))}


def home_clearance(solver, q):
    frames = solver.frames(q)
    distances = []
    for side in SIDES:
        links = [f'{side}_{link}' for link in ('shoulder_Link', 'upperarm2_Link', 'forearm2_Link', 'flange')]
        for i, link in enumerate(links):
            for alpha in ((0., .25, .5, .75) if i < 3 else (0.,)):
                start, end = frames[link], frames[links[min(i + 1, 3)]]
                point = (1 - alpha) * start[:3, 3] + alpha * end[:3, 3]
                for box, (center, half_extent) in BOXES_MM.items():
                    frame = frames[box]
                    local = frame[:3, :3].T @ (point - frame[:3, 3])
                    delta = np.abs(local - center) - half_extent - 60.
                    distance = float(np.linalg.norm(np.maximum(delta, 0.)) + min(0., max(delta)))
                    distances.append(dict(mm=distance, link=link, alpha=alpha, box=box))
    minimum = min(row['mm'] for row in distances)
    hold = solver.solve_arms({s: solver.forward(s, q) for s in SIDES}, q)
    return dict(minimum_mm=minimum, inside_activation_band=[r for r in distances if r['mm'] < 40.],
                violates_minimum=minimum < 10. - .01, hold_native_clearance_mm=hold['clearance_mm'],
                hold_iterations=hold['iterations'], hold_status=hold['status'])


def distribution(values):
    return dict(zip(('p50', 'p95', 'max'), np.percentile(values, [50, 95, 100]).tolist()))


def solve_audit(solver, body):
    # Same starting high torso for each Home. Both distant wrists lie beyond
    # the fixed shoulder's 290+310 mm reach, even before enforcing joint limits.
    far = body_pose(body, 65, 10)
    far[:14] = np.radians([45, 60, -80, 25, 75, 10, 0, -45, 60, 80, 25, -75, 10, 0])
    far_goals = {s: solver.forward(s, far) for s in SIDES}
    far_frames = solver.frames(far)
    records = {}
    for name, arms in homes().items():
        seed = body_pose(body, 100)
        seed[:14] = arms
        small = seed.copy()
        small[[0, 7]] += .006
        local_goals = {s: solver.forward(s, small) for s in SIDES}
        frames = solver.frames(seed)
        reach = {s: float(np.linalg.norm(far_frames[s + '_waist2_Link'][:3, 3] -
                                         frames[s + '_shoulder_Link'][:3, 3])) for s in SIDES}
        records[name] = dict(initial_box=home_clearance(solver, seed), far_wrist_from_shoulder_mm=reach,
                             cases={})
        cases = [('near_arms', solver.solve_arms, local_goals), ('far_arms', solver.solve_arms, far_goals),
                 ('far_body', solver.solve_body, far_goals)]
        rows = {key: [] for key, _, _ in cases}
        for repeat in range(21):
            for key, solve, goals in cases:
                start = perf_counter_ns()
                result = solve(goals, seed)
                wall_ms = (perf_counter_ns() - start) / 1e6
                if repeat:
                    rows[key].append(dict(wall_ms=wall_ms, native_ms=result['native_ms'],
                                          iterations=result['iterations'], qp_iterations=result['qp_iterations'],
                                          success=result['success'], status=result['status'],
                                          errors_mm_deg=result['errors_mm_deg'],
                                          max_torso_change_deg=float(np.max(np.abs(np.degrees(
                                              result['q'][15:] - seed[15:]))))))
        for key, samples in rows.items():
            records[name]['cases'][key] = dict(
                counts=dict(Counter(r['status'] for r in samples)), success=sum(r['success'] for r in samples),
                samples=len(samples), errors_mm_deg=samples[-1]['errors_mm_deg'],
                max_torso_change_deg=max(r['max_torso_change_deg'] for r in samples),
                **{field: distribution([r[field] for r in samples])
                   for field in ('wall_ms', 'native_ms', 'iterations', 'qp_iterations')})
    return records


def main():
    tools = {mode: tool_config(mode) for mode in ('flange', 'taccap')}
    solvers = {mode: Art7FairinoQP(config) for mode, config in tools.items()}
    model = solver_reference()
    arm, chain = forward_audit(solvers, tools, model)
    mapping = mapping_audit(solvers['flange'], model)
    limits = height_limits(solvers['flange'], model.body)
    solves = solve_audit(solvers['taccap'], model.body)
    gates = {mode: item['position_mm'] <= POSITION_MM and item['orientation_deg'] <= ORIENTATION_DEG
             for mode, item in arm.items()}
    gates['full_chain'] = chain['position_mm'] <= 1e-8 and chain['orientation_deg'] <= 1e-5
    gates['mapping'] = (mapping['max_orientation_error_deg'] <= ORIENTATION_DEG
                        and max(mapping['max_height_error_mm'], mapping['max_horizontal_drift_mm']) <= POSITION_MM)
    gates['solve'] = all(
        all(run['cases'][key]['success'] == run['cases'][key]['samples'] for key in ('near_arms', 'far_body'))
        and run['cases']['far_arms']['success'] == 0
        and min(run['far_wrist_from_shoulder_mm'].values()) > 601.
        and run['cases']['far_body']['max_torso_change_deg'] > 0.
        for run in solves.values())
    report = dict(rng_seed=RNG_SEED, random_arm_pairs=RANDOM_PAIRS, torso_states=TORSOS,
                  threshold_mm_deg=[POSITION_MM, ORIENTATION_DEG], tool_configs=tools,
                  python=sys.version, numpy=np.__version__, cpu_affinity=len(os.sched_getaffinity(0)),
                  load_average=os.getloadavg(), arm=arm, full_chain=chain, mapping=mapping,
                  out_of_range_height=limits, solves=solves, gates=gates,
                  timing='taccap; 1 excluded warmup then 20 interleaved repeats per case, fixed Home seed; '
                         'wall includes adapter and final TCP FK, native includes only C++ solve_ik')
    path = CACHE / 'evaluation.json'
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(path)
    print(json.dumps(gates))
    return 0 if all(gates.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
