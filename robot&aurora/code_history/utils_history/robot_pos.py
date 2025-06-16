from typing import List, Tuple
from .probe import Vector

class Rotation:
    def __init__(self, roll: float, pitch: float, yaw: float) -> None:
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    
    def to_str(self) -> str:
        return "({}, {}, {})".format(self.roll, self.pitch, self.yaw)

class Robot:
    def __init__(self, code: int, pos: Vector, rot: Rotation) -> None:
        self.code = code
        self.pos = pos
        self.rot = rot

def generate_robot(
    a: Tuple[int, List[float]]
) -> Robot:
    code = a[0]
    position = a[1]
    return Robot(
        code,
        Vector(position[0], position[1], position[2]),
        Rotation(position[3], position[4], position[5])
    )