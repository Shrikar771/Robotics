package team5427.frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.drivers.ComplexGearRatio;
import team5427.lib.motors.MotorConfiguration;
import team5427.lib.motors.MotorConfiguration.IdleState;
import team5427.lib.motors.MotorConfiguration.MotorMode;
import team5427.lib.motors.MotorUtil;

public final class IntakeConstants {
  public static MotorConfiguration kPivotMotorConfiguration = new MotorConfiguration();
  public static MotorConfiguration kRollerMotorConfiguration = new MotorConfiguration();

  public static final ComplexGearRatio kPivotGearRatio = new ComplexGearRatio();
  public static final ComplexGearRatio kRollerGearRatio = new ComplexGearRatio();

  public static final CANDeviceId kPivotCanDeviceId = new CANDeviceId(2);
  public static final CANDeviceId kRollerCanDeviceId = new CANDeviceId(3);

  static {
    kPivotMotorConfiguration.gearRatio = kPivotGearRatio;
    kPivotMotorConfiguration.isArm = true;
    kPivotMotorConfiguration.idleState = IdleState.kBrake;
    kPivotMotorConfiguration.isInverted =
        false; // If motor and mechanism move the same way, then it is not inverted
    kPivotMotorConfiguration.mode = MotorMode.kServo;
    kPivotMotorConfiguration.withFOC = true;

    kPivotMotorConfiguration.maxVelocity =
        kPivotMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenX60FOC_MaxRPM);
    kPivotMotorConfiguration.maxAcceleration =
        kPivotMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenX60FOC_MaxRPM) * 2.0;

    kPivotMotorConfiguration.altV =
        kPivotMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenX60FOC_MaxRPM) / 2.0;
    kPivotMotorConfiguration.altA = kPivotMotorConfiguration.maxAcceleration;
    kPivotMotorConfiguration.altJ = 1000.0;

    kPivotMotorConfiguration.currentLimit = 40;
    kPivotMotorConfiguration.kA = 0.0;
    kPivotMotorConfiguration.kD = 0.0;
    kPivotMotorConfiguration.kFF = 0.0;
    kPivotMotorConfiguration.kP = 1.0;
    kPivotMotorConfiguration.kG = 0.0;
    kPivotMotorConfiguration.kS = 0.0;
    kPivotMotorConfiguration.kV = 0.0;
    kPivotMotorConfiguration.kI = 0.0;
  }

  static {
    kRollerMotorConfiguration.gearRatio = kRollerGearRatio;
    kRollerMotorConfiguration.finalDiameterMeters = Units.metersToInches(2.0);
    kRollerMotorConfiguration.isArm = false;
    kRollerMotorConfiguration.idleState = IdleState.kBrake;
    kRollerMotorConfiguration.isInverted =
        false; // If motor and mechanism move the same way, then it is not inverted
    kRollerMotorConfiguration.mode = MotorMode.kFlywheel;
    kRollerMotorConfiguration.withFOC = true;

    kRollerMotorConfiguration.maxVelocity =
        kRollerMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenX60FOC_MaxRPM);
    kRollerMotorConfiguration.maxAcceleration =
        kRollerMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenX60FOC_MaxRPM) * 2.0;

    kRollerMotorConfiguration.currentLimit = 40;
    kRollerMotorConfiguration.kA = 0.0;
    kRollerMotorConfiguration.kD = 0.0;
    kRollerMotorConfiguration.kFF = 0.0;
    kRollerMotorConfiguration.kP = 1.0;
    kRollerMotorConfiguration.kG = 0.0;
    kRollerMotorConfiguration.kS = 0.0;
    kRollerMotorConfiguration.kV = 0.0;
    kRollerMotorConfiguration.kI = 0.0;
  }

  public static final Rotation2d kPivotMaxRotation = Rotation2d.fromDegrees(120);
  public static final Rotation2d kPivotMinimumRotation = Rotation2d.fromDegrees(0);
  public static final Rotation2d kPivotStartingRotation = Rotation2d.fromDegrees(0);
}
