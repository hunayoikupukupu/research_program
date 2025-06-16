from typing import List, Tuple


class Quaternion:
    def __init__(self, w: float, x: float, y: float, z: float) -> None:
        self.w = w
        self.x = x
        self.y = y
        self.z = z


class Vector:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z

    def to_str(self) -> str:
        return "({}, {}, {})".format(self.x, self.y, self.z)


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


def generate_probe(
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