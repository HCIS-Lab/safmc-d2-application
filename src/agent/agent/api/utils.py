from dataclasses import dataclass

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