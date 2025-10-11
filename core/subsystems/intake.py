from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from rev import SparkMax, SparkBaseConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class Intake(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake

    self._motor = SparkMax(self._constants.kMotorCANId, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkBaseConfig()
    (self._sparkConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kMotorCurrentLimit)
      .inverted(False))
    utils.setSparkConfig(
      self._motor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

    self._motor2 = SparkMax(13, SparkBase.MotorType.kBrushless)
    self._sparkConfig2 = SparkBaseConfig()
    (self._sparkConfig2
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kMotorCurrentLimit))
    self._sparkConfig2.follow(12, False)
    utils.setSparkConfig(
      self._motor2.configure(
        self._sparkConfig2,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

  def periodic(self) -> None:
    self._updateTelemetry()
      
  def runIntake(self) -> Command:
    return self.runEnd(
      lambda: self._motor.set(-self._constants.kMotorIntakeSpeed),
      lambda: self.reset()
    ).withName("Intake:Run")

  def runEject(self) -> Command:
    return self.startEnd(
      lambda: self._motor.set(self._constants.kMotorEjectSpeed),
      lambda: self.reset()
    ).withTimeout(
      self._constants.kEjectTimeout
    ).withName("Intake:Eject")

  def isEnabled(self) -> bool:
    return self._motor.get() != 0
  
  def reset(self) -> None:
    self._motor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Intake/IsEnabled", self.isEnabled())
    # SmartDashboard.putNumber("Robot/Intake/Current", self._gripperMotor.getOutputCurrent())
 