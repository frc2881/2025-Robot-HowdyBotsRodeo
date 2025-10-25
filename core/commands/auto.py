from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from lib import logger, utils
if TYPE_CHECKING: from core.robot import RobotCore
import core.constants as constants

class AutoPath(Enum):
  Move1 = auto()

class Auto:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

    self._auto = cmd.none()

    self._autos = SendableChooser()
    self._autos.setDefaultOption("None", cmd.none)
    
    self._autos.addOption("[1]", self.auto_1)

    self._autos.onChange(lambda auto: self.set(auto()))
    SmartDashboard.putData("Robot/Auto", self._autos)

  def get(self) -> Command:
    return self._auto
  
  def set(self, auto: Command) -> None:
    self._auto = auto
  
  def _moveForwards(self) -> Command:
    return (
      self._robot.drive.drive(lambda: 1.0, lambda: 0.0).withTimeout(3.4).andThen(self._robot.drive.reset())
    )
  
  def _moveBackwards(self) -> Command:
    return (
      self._robot.drive.drive(lambda: -1.0, lambda: 0.0).withTimeout(2.5).andThen(self._robot.drive.reset())
    )

  def auto_1(self) -> Command:
    return (
      self._moveForwards()
      .andThen(cmd.waitSeconds(2.0))
      .andThen(self._robot.intake.eject())
      .deadlineFor(self._robot.arm.setPosition(constants.Subsystems.Arm.kArmPositionScoreSalvage))
      .andThen(self._moveBackwards())
    ).withName("Auto:[1]")
