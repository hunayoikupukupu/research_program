from typing import List, Tuple

class Vector:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z

class Quaternion:
    def __init__(self, w: float, x: float, y: float, z: float) -> None:
        self.w = w
        self.x = x
        self.y = y
        self.z = z

class Probe:
    def __init__(
        self,
        port_number: int,
        time_stamp: float,
        frame_numbers: float,
        pos: Vector,
        quat: Quaternion,
        quality: float,
    ) -> None:
        self.port_number = port_number
        self.time_stamp = time_stamp
        self.frame_numbers = frame_numbers
        self.pos = pos
        self.quat = quat
        self.quality = quality


def generateProbe(
    frame: Tuple[
        List[int],
        List[float],
        List[float],
        List[List[float]],
        List[float],
    ]
) -> List[Probe]:
    port_numbers = frame[0]
    time_stamps = frame[1]
    frame_numbers = frame[2]
    trackings = frame[3]
    qualities = frame[4]

    port_num = len(port_numbers)

    probes: List[Probe] = []
    for i in range(port_num):
        tracking = trackings[i][0]
        probes.append(
            Probe(
                port_numbers[i],
                time_stamps[i],
                frame_numbers[i],
                Vector(tracking[4], tracking[5], tracking[6]),
                Quaternion(tracking[0], tracking[1], tracking[2], tracking[3]),
                qualities[i],
            )
        )

    return probes

class EulerRotation:
    def __init__(self, roll: float, pitch: float, yaw: float) -> None:
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class RobotArm:
    def __init__(self, code: int, pos: Vector, rot: EulerRotation) -> None:
        self.code = code
        self.pos = pos
        self.rot = rot

def generateRobotArm(
    armPose: Tuple[int, List[float]]
) -> RobotArm:
    code = armPose[0]
    position = armPose[1]
    return RobotArm(
        code,
        Vector(position[0], position[1], position[2]),
        EulerRotation(position[3], position[4], position[5])
    )

class AxisAngle:
    def __init__(self, rx: float, ry: float, rz: float) -> None:
        self.rx = rx
        self.ry = ry
        self.rz = rz

class RobotArmAxisAngle:
    def __init__(self, code: int, pos: Vector, rot: AxisAngle) -> None:
        self.code = code
        self.pos = pos
        self.rot = rot

def generateRobotArmAxisAngle(
    armPose: Tuple[int, List[float]]
) -> RobotArmAxisAngle:
    code = armPose[0]
    position = armPose[1]
    return RobotArmAxisAngle(
        code,
        Vector(position[0], position[1], position[2]),
        AxisAngle(position[3], position[4], position[5])
    )