import math
from dataclasses import dataclass

from common.decorators import staticproperty


@dataclass
class NEDCoordinate:

    x: float  # n
    y: float  # e
    z: float  # d

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self) -> str:
        return f"NED(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"

    def __neg__(self) -> 'NEDCoordinate':
        return NEDCoordinate(-self.x, -self.y, -self.z)

    def __add__(self, other: 'NEDCoordinate') -> 'NEDCoordinate':
        if not isinstance(other, NEDCoordinate):
            raise TypeError(
                f"Unsupported operand type(s) for +: 'NEDCoordinate' and '{type(other).__name__}'")
        return NEDCoordinate(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: 'NEDCoordinate') -> 'NEDCoordinate':
        if not isinstance(other, NEDCoordinate):
            raise TypeError(
                f"Unsupported operand type(s) for +: 'NEDCoordinate' and '{type(other).__name__}'")
        return NEDCoordinate(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> 'NEDCoordinate':
        if not isinstance(scalar, (int, float)):
            raise TypeError(
                f"Unsupported operand type(s) for *: 'NEDCoordinate' and '{type(scalar).__name__}'")
        return NEDCoordinate(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> 'NEDCoordinate':
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> 'NEDCoordinate':
        if not isinstance(scalar, (int, float)):
            raise TypeError(
                f"Unsupported operand type(s) for *: 'NEDCoordinate' and '{type(scalar).__name__}'")
        if scalar == 0:
            raise ValueError("Division by zero is not allowed.")
        return NEDCoordinate(self.x / scalar, self.y / scalar, self.z / scalar)

    @property
    def magnitude(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    @property
    def normalized(self) -> 'NEDCoordinate':
        mag = self.magnitude
        if mag == 0:
            raise ValueError("Cannot normalize a zero vector.")
        return self / mag

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
