"""ART7 nominal mounting transforms shared by keyboard planning and ROS checks.

Pure Python, so this module also runs in the remote in-memory source bundle.
Parameters are copied verbatim from the manufacturer's full-body URDF; tests
compare them with that source and with the PICO mapping. All output lengths mm.
"""
from tcp_core import identity, multiply, rotation, target_matrix

CHEST_FRAME_ID = 'art7-both-chest-urdf-20261030-v1'
SOURCE_ARCHIVE = 'RD53-轮足机器人URDF和模型-v1.0-20261030'
SOURCE_SHA256 = '96d4b4fb6e6a584bc55e1e9f1cf538ff9b02cc2093f1cda1cc983cd48199be9c'
# body_Link -> arm base: origin xyz [m], rpy [rad]; then arm-base -> DH XY [m].
MOUNTS = {
    'left': ((-.048, .00085882, .32961), (-1.5708, .7854, 1.5708), (-.00088174, .00033281)),
    'right': ((.048, -.00038815, .33086), (1.5708, .7854, 1.5708), (.00088174, -.00033281)),
}
# Chest origin = body_Link origin; chest X/Y/Z = body +Y/-X/+Z.
CHEST_FROM_BODY = [[0., 1., 0., 0.], [-1., 0., 0., 0.],
                   [0., 0., 1., 0.], [0., 0., 0., 1.]]


def _mounts():
    result = {}
    for arm, (xyz, rpy, xy) in MOUNTS.items():
        roll, pitch, yaw = rpy
        body_from_base = multiply(multiply(rotation(2, yaw), rotation(1, pitch)), rotation(0, roll))
        for i in range(3): body_from_base[i][3] = xyz[i]*1000.
        base_from_dh = identity()
        # d1=187 mm is already in vendor FK. Include only CAD's XY origin shift.
        for i in range(2): base_from_dh[i][3] = xy[i]*1000.
        result[arm] = multiply(multiply(CHEST_FROM_BODY, body_from_base), base_from_dh)
    return result


CHEST_FROM_DH = _mounts()


def _mount(arm):
    if arm not in CHEST_FROM_DH:
        raise ValueError('未知机械臂：'+str(arm))
    return CHEST_FROM_DH[arm]


def tcp_in_chest(arm, tcp_in_dh):
    return multiply(_mount(arm), tcp_in_dh)


def tcp_in_base(arm, tcp_in_chest_frame):
    mount = _mount(arm)
    inverse = identity()
    for i in range(3):
        for j in range(3): inverse[i][j] = mount[j][i]
        inverse[i][3] = -sum(mount[j][i]*mount[j][3] for j in range(3))
    return multiply(inverse, tcp_in_chest_frame)


def chest_target_in_base(arm, start_in_base, axis, direction, step):
    """Construct a chest-axis increment at the flange, then convert for IK."""
    chest_start = tcp_in_chest(arm, start_in_base)
    chest_goal = target_matrix(chest_start, axis, direction, step)
    return tcp_in_base(arm, chest_goal)
