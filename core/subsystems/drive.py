from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard, SendableChooser
from wpilib.drive import DifferentialDrive
from wpimath import units
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveWheelSpeeds
from pathplannerlib.util import DriveFeedforwards
from lib import logger, utils
from lib.classes import State, MotorIdleMode, SpeedMode, DriveOrientation, TargetAlignmentMode, DifferentialModuleLocation, DifferentialDriveModulePositions
from lib.components.differential_module import DifferentialModule
import core.constants as constants

class Drive(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], units.degrees]
    ) -> None:
    super().__init__()
    self._getGyroHeading = getGyroHeading
    
    self._constants = constants.Subsystems.Drive

    self._modules = tuple(DifferentialModule(c) for c in self._constants.kDifferentialModuleConfigs)

    self._drivetrain = DifferentialDrive(
      self._modules[DifferentialModuleLocation.Left].getMotorController(),
      self._modules[DifferentialModuleLocation.Right].getMotorController()
    )

    self._inputXFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputYFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputRotationFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)

    self._driftCorrectionState = State.Stopped
    self._driftCorrectionController = PIDController(*self._constants.kDriftCorrectionConstants.rotationPID)
    self._driftCorrectionController.setTolerance(*self._constants.kDriftCorrectionConstants.rotationTolerance)
    self._driftCorrectionController.enableContinuousInput(-180.0, 180.0)

    self._targetAlignmentState = State.Stopped
    self._targetAlignmentPose: Pose3d = None
    self._targetAlignmentTranslationXController = ProfiledPIDController(
      *self._constants.kTargetAlignmentConstants.translationPID, 
      TrapezoidProfile.Constraints(self._constants.kTargetAlignmentConstants.translationMaxVelocity, self._constants.kTargetAlignmentConstants.translationMaxAcceleration)
    )
    self._targetAlignmentTranslationXController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentTranslationYController = ProfiledPIDController(
      *self._constants.kTargetAlignmentConstants.translationPID, 
      TrapezoidProfile.Constraints(self._constants.kTargetAlignmentConstants.translationMaxVelocity, self._constants.kTargetAlignmentConstants.translationMaxAcceleration)
    )
    self._targetAlignmentTranslationYController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentRotationController = ProfiledPIDController(
      *self._constants.kTargetAlignmentConstants.rotationPID, 
      TrapezoidProfile.Constraints(self._constants.kTargetAlignmentConstants.rotationMaxVelocity, self._constants.kTargetAlignmentConstants.rotationMaxAcceleration)
    )
    self._targetAlignmentRotationController.setTolerance(*self._constants.kTargetAlignmentConstants.rotationTolerance)
    self._targetAlignmentRotationController.enableContinuousInput(-180.0, 180.0)

    self._speedMode: SpeedMode = SpeedMode.Competition
    speedMode = SendableChooser()
    speedMode.setDefaultOption(SpeedMode.Competition.name, SpeedMode.Competition)
    speedMode.addOption(SpeedMode.Demo.name, SpeedMode.Demo)
    speedMode.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
    SmartDashboard.putData("Robot/Drive/SpeedMode", speedMode)

    self._orientation: DriveOrientation = DriveOrientation.Robot
    orientation = SendableChooser()
    orientation.setDefaultOption(DriveOrientation.Field.name, DriveOrientation.Field)
    orientation.addOption(DriveOrientation.Robot.name, DriveOrientation.Robot)
    orientation.onChange(lambda orientation: setattr(self, "_orientation", orientation))
    SmartDashboard.putData("Robot/Drive/Orientation", orientation)

    self._driftCorrection: State = State.Disabled
    driftCorrection = SendableChooser()
    driftCorrection.setDefaultOption(State.Enabled.name, State.Enabled)
    driftCorrection.addOption(State.Disabled.name, State.Disabled)
    driftCorrection.onChange(lambda driftCorrection: setattr(self, "_driftCorrection", driftCorrection))
    SmartDashboard.putData("Robot/Drive/DriftCorrection", driftCorrection)

    idleMode = SendableChooser()
    idleMode.setDefaultOption(MotorIdleMode.Brake.name, MotorIdleMode.Brake)
    idleMode.addOption(MotorIdleMode.Coast.name, MotorIdleMode.Coast)
    idleMode.onChange(lambda idleMode: self._setIdleMode(idleMode))
    SmartDashboard.putData("Robot/Drive/IdleMode", idleMode)

    SmartDashboard.putNumber("Robot/Drive/Chassis/RobotLength", self._constants.kRobotLength)
    SmartDashboard.putNumber("Robot/Drive/Chassis/RobotWidth", self._constants.kRobotWidth)

  def periodic(self) -> None:
    self._updateTelemetry()

  def drive(self, getInputLeft: Callable[[], units.percent], getInputRight: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._drive(getInputLeft(), getInputRight())
    ).withName("Drive:Drive")

  def _drive(self, speed: float, rotation: float) -> None:
    self._drivetrain.arcadeDrive(speed, rotation, True)

  def setChassisSpeeds(self, chassisSpeeds: ChassisSpeeds, driveFeedforwards: DriveFeedforwards = None) -> None:
    self._setModuleStates(chassisSpeeds)

  def getChassisSpeeds(self) -> ChassisSpeeds:
    return self._constants.kDriveKinematics.toChassisSpeeds(
      DifferentialDriveWheelSpeeds(
        self._modules[DifferentialModuleLocation.Left].getVelocity(), 
        self._modules[DifferentialModuleLocation.Right].getVelocity()
      )
    )

  def getModulePositions(self) -> DifferentialDriveModulePositions:
    return DifferentialDriveModulePositions(
      self._modules[DifferentialModuleLocation.Left].getPosition(),
      self._modules[DifferentialModuleLocation.Right].getPosition()
    )

  def _setModuleStates(self, chassisSpeeds: ChassisSpeeds) -> None: 
    wheelSpeeds = self._constants.kDriveKinematics.toWheelSpeeds(chassisSpeeds)
    self._drivetrain.tankDrive(wheelSpeeds.left, wheelSpeeds.right)
    if self._targetAlignmentState != State.Running:
      if chassisSpeeds.vx != 0 or chassisSpeeds.vy != 0:
        self._resetTargetAlignment()

  def _getModuleStates(self) -> tuple[float, ...]:
    return tuple(m.getVelocity() for m in self._modules)

  def _setIdleMode(self, idleMode: MotorIdleMode) -> None:
    for m in self._modules: m.setIdleMode(idleMode)
    SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name)

  def alignToTarget(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d], targetAlignmentMode: TargetAlignmentMode) -> Command:
    return self.startRun(
      lambda: self._initTargetAlignment(getRobotPose(), getTargetPose(), targetAlignmentMode),
      lambda: self._runTargetAlignment(getRobotPose(), targetAlignmentMode)
    ).until(
      lambda: self._targetAlignmentState == State.Completed
    ).finallyDo(
      lambda end: self._endTargetAlignment()
    ).withName("Drive:AlignToTarget")
  
  def _initTargetAlignment(self, robotPose: Pose2d, targetAlignmentPose: Pose3d, targetAlignmentMode: TargetAlignmentMode) -> None:
    self._resetTargetAlignment()
    self._targetAlignmentPose = targetAlignmentPose
    self._targetAlignmentState = State.Running
    self._targetAlignmentTranslationXController.reset(0)
    self._targetAlignmentTranslationXController.setGoal(0)
    self._targetAlignmentTranslationYController.reset(0)
    self._targetAlignmentTranslationYController.setGoal(0)
    self._targetAlignmentRotationController.reset(robotPose.rotation().degrees())
    self._targetAlignmentRotationController.setGoal(
      targetAlignmentPose.toPose2d().rotation().degrees() + self._constants.kTargetAlignmentConstants.rotationTranslationModeOffset
      if targetAlignmentMode == TargetAlignmentMode.Translation else
      utils.wrapAngle(utils.getTargetHeading(robotPose, targetAlignmentPose) + self._constants.kTargetAlignmentConstants.rotationHeadingModeOffset)
    )

  def _runTargetAlignment(self, robotPose: Pose2d, targetAlignmentMode: TargetAlignmentMode) -> None:
    speedTranslationX = 0
    speedTranslationY = 0
    speedRotation = 0
    if speedRotation == 0 and speedTranslationX == 0 and speedTranslationY == 0:
      self._targetAlignmentState = State.Completed

  def _endTargetAlignment(self) -> None:
    self._setModuleStates(ChassisSpeeds())
    if self._targetAlignmentState != State.Completed:
      self._targetAlignmentState = State.Stopped

  def isAlignedToTarget(self) -> bool:
    return self._targetAlignmentState == State.Completed
  
  def _resetTargetAlignment(self) -> None:
    self._targetAlignmentPose = None
    self._targetAlignmentState = State.Stopped

  def reset(self) -> None:
    self.setChassisSpeeds(ChassisSpeeds())
    self._resetTargetAlignment()
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Drive/IsAlignedToTarget", self.isAlignedToTarget())
    SmartDashboard.putString("Robot/Drive/TargetAlignmentState", self._targetAlignmentState.name)
