package team5427.frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInput {
    // Initialized pivot motor rotation to 0 degrees
    public Rotation2d pivotMotorRotation = Rotation2d.kZero;
    // Initialized pivot motor's angular velocity to 0 initially
    public AngularVelocity pivotMotorAngularVelocity = RotationsPerSecond.of(0.0);
    // Initialized pivot motor's angular acceleratiopn to a random constant since acceleration
    // cannot be simply 0
    public AngularAcceleration pivotMotorAngularAcceleration = RotationsPerSecondPerSecond.of(2.0);

    // Initialized the roller motor's linear velocity
    public LinearVelocity rollerMotorLinearVelocity = MetersPerSecond.of(0.0);
    // Initialized roller motor's linear acceleration
    public LinearAcceleration rollerMotorLinearAcceleration = MetersPerSecondPerSecond.of(2.0);
    // Initialize roller motor's angular velocity
    public AngularVelocity rollerMotorAngularVelocity = RotationsPerSecond.of(0.0);
    // Initialize roller motor's angular acceleration to a random constant that is reasonable
    public AngularAcceleration rollerMotorAngularAcceleration = RotationsPerSecondPerSecond.of(2.0);

    // sets currents
    public Current pivotMotorCurrent = Amps.of(0.0);
    public Current rollerMotorCurrent = Amps.of(0.0);
    // sets voltages
    public Voltage pivotMotorVoltage = Volts.of(0.0);
    public Voltage rollerMotorVoltage = Volts.of(0.0);
    // declares if motors are connected
    public boolean pivotMotorConnected = false;
    public boolean rollerMotorConnected = false;

    public Temperature rollerMotorTemperature = Celsius.of(0.0);
    public Temperature pivotMotorTemperature = Celsius.of(0.0);

    public boolean pivotMotorDisabled = false;
    public boolean rollerMotorDisabled = false;
  }

  public default void updateInputs(IntakeIOInput input) {}

  public default void setPivotRotation(Rotation2d rotation) {}

  public default void setRollerSpeed(LinearVelocity velocity) {}

  public default void setPivotRotation(Angle rotation) {}

  public default void setRolerSpeed(AngularVelocity velocity) {}

  public default void resetPivotEncoderPosition(Rotation2d resetAngle) {}

  public default void disableRollerMotor(boolean shouldDisable) {}

  public default void disablePivotMotor(boolean shouldDisable) {}
}
