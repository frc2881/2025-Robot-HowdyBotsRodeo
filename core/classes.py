from enum import Enum, auto
from dataclasses import dataclass
from wpimath.geometry import Pose3d

class TargetType(Enum):
  Default = auto()

@dataclass(frozen=True, slots=True)
class Target():
  type: TargetType
  pose: Pose3d
