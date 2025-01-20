import math
from dataclasses import dataclass

from common.decorators import staticproperty


@dataclass
class NEDCoordinate:
    # TODO: https://docs.unity3d.com/6000.0/Documentation/ScriptReference/Vector3.html

    x: float
    y: float
    z: float

    def __str__(self) -> str:
        return f"NED(x={self.x}, y={self.y}, z={self.z})"

    def __add__(self, other: 'NEDCoordinate') -> 'NEDCoordinate':
        return NEDCoordinate(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: 'NEDCoordinate') -> 'NEDCoordinate':
        return NEDCoordinate(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> 'NEDCoordinate':
        return NEDCoordinate(self.x * scalar, self.y * scalar, self.z * scalar)

    @staticmethod
    def distance(coord1: 'NEDCoordinate', coord2: 'NEDCoordinate') -> float:
        return math.sqrt((coord1.x - coord2.x) ** 2 + (coord1.y - coord2.y) ** 2 + (coord1.z - coord2.z) ** 2)

    @staticproperty
    def north() -> 'NEDCoordinate':
        return NEDCoordinate(1, 0, 0)

    @staticproperty
    def east() -> 'NEDCoordinate':
        return NEDCoordinate(0, 1, 0)

    @staticproperty
    def down() -> 'NEDCoordinate':
        return NEDCoordinate(0, 0, 1)
