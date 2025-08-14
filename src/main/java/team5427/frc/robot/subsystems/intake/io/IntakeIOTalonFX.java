package team5427.frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants;
import team5427.frc.robot.subsystems.intake.IntakeConstants;
import team5427.lib.motors.SteelTalonFX;

public class IntakeIOTalonFX implements IntakeIO {
  private SteelTalonFX rollerMotor;
  private SteelTalonFX pivotMotor;

  private StatusSignal<Angle> pivotMotorPosition;
  private StatusSignal<AngularVelocity> pivotMotorAngularVelocity;
  private StatusSignal<AngularAcceleration> pivotMotorAngularAcceleration;

  private StatusSignal<AngularVelocity> rollerMotorAngularVelocity;
  private StatusSignal<AngularAcceleration> rollerMotorAngularAcceleration;

  private StatusSignal<Current> rollerMotorCurrent;
  private StatusSignal<Current> pivotMotorCurrent;

  private StatusSignal<Voltage> rollerMotorVoltage;
  private StatusSignal<Voltage> pivotMotorVoltage;

  private StatusSignal<Temperature> rollerMotorTemp;
  private StatusSignal<Temperature> pivotMotorTemp;

  private boolean isRollerMotorDisabled = false;
  private boolean isPivotMotorDisabled = false;

  public boolean pivotMotorDisabled = false;
  public boolean rollerMotorDisabled = false;

  public IntakeIOTalonFX() {
    rollerMotor = new SteelTalonFX(IntakeConstants.kRollerCanDeviceId);
    pivotMotor = new SteelTalonFX(IntakeConstants.kPivotCanDeviceId);

    rollerMotor.apply(IntakeConstants.kRollerMotorConfiguration);
    pivotMotor.apply(IntakeConstants.kPivotMotorConfiguration);

    pivotMotor.setEncoderPosition(IntakeConstants.kPivotStartingRotation);
    rollerMotor.setEncoderPosition(0.0);

    pivotMotorPosition = pivotMotor.getTalonFX().getPosition();
    pivotMotorAngularVelocity = pivotMotor.getTalonFX().getVelocity();
    pivotMotorAngularAcceleration = pivotMotor.getTalonFX().getAcceleration();

    rollerMotorAngularVelocity = rollerMotor.getTalonFX().getVelocity();
    rollerMotorAngularAcceleration = rollerMotor.getTalonFX().getAcceleration();

    rollerMotorCurrent = rollerMotor.getTalonFX().getStatorCurrent();
    pivotMotorCurrent = pivotMotor.getTalonFX().getStatorCurrent();

    rollerMotorVoltage = rollerMotor.getTalonFX().getMotorVoltage();
    pivotMotorVoltage = pivotMotor.getTalonFX().getMotorVoltage();

    rollerMotorTemp = rollerMotor.getTalonFX().getDeviceTemp();
    pivotMotorTemp = pivotMotor.getTalonFX().getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kHighPriorityUpdateFrequency, pivotMotorPosition, rollerMotorAngularVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kMediumPriorityUpdateFrequency,
        pivotMotorCurrent,
        pivotMotorAngularAcceleration,
        rollerMotorCurrent,
        rollerMotorAngularAcceleration,
        rollerMotorAngularVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kLowPriorityUpdateFrequency,
        pivotMotorVoltage,
        pivotMotorTemp,
        rollerMotorVoltage,
        rollerMotorTemp);
  }

  public void updateInputs(IntakeIOInput inputs) {
    inputs.rollerMotorConnected = rollerMotor.getTalonFX().isConnected();
    inputs.pivotMotorConnected = pivotMotor.getTalonFX().isConnected();

    inputs.rollerMotorDisabled = isRollerMotorDisabled;
    inputs.pivotMotorDisabled = isPivotMotorDisabled;

    BaseStatusSignal.refreshAll(pivotMotorPosition, rollerMotorAngularVelocity);
    BaseStatusSignal.refreshAll(
        rollerMotorCurrent,
        pivotMotorCurrent,
        pivotMotorAngularAcceleration,
        rollerMotorAngularAcceleration,
        rollerMotorAngularVelocity);

    BaseStatusSignal.refreshAll(
        pivotMotorVoltage, pivotMotorTemp, rollerMotorVoltage, rollerMotorTemp);

    inputs.pivotMotorRotation =
        Rotation2d.fromRotations(pivotMotorPosition.getValue().in(Rotation));
    inputs.pivotMotorAngularVelocity = pivotMotorAngularVelocity.getValue();
    inputs.pivotMotorAngularAcceleration = pivotMotorAngularAcceleration.getValue();
    inputs.pivotMotorCurrent = pivotMotorCurrent.getValue();
    inputs.pivotMotorVoltage = pivotMotorVoltage.getValue();
    inputs.pivotMotorTemperature = pivotMotorTemp.getValue();

    inputs.rollerMotorAngularVelocity = rollerMotorAngularVelocity.getValue();
    inputs.rollerMotorAngularAcceleration = rollerMotorAngularAcceleration.getValue();
    inputs.rollerMotorTemperature = rollerMotorTemp.getValue();
    inputs.rollerMotorCurrent = rollerMotorCurrent.getValue();
    inputs.rollerMotorLinearVelocity =
        MetersPerSecond.of(rollerMotor.getEncoderVelocity(rollerMotorAngularVelocity));
    inputs.rollerMotorLinearAcceleration =
        MetersPerSecondPerSecond.of(
            rollerMotor.getEncoderAcceleration(pivotMotorAngularAcceleration));
    inputs.rollerMotorVoltage = rollerMotorVoltage.getValue();
  }

  public void setPivotRotation(Rotation2d rotation) {
    if (isPivotMotorDisabled) {
      pivotMotor.setSetpoint(rotation);
    }
  }

  public void setRollerSpeed(LinearVelocity velocity) {
    if (isRollerMotorDisabled) {
      rollerMotor.setSetpoint(velocity);
    }
  }

  public void setPivotRotation(Angle rotation) {
    if (isPivotMotorDisabled) {
      pivotMotor.setSetpoint(rotation);
    }
  }

  public void setRollerSpeed(AngularVelocity velocity) {
    if (isRollerMotorDisabled) {
      rollerMotor.setSetpoint(velocity);
    }
  }

  public void resetPivotEncoderPosition(Rotation2d resetAngle) {
    if (isPivotMotorDisabled) {
      pivotMotor.setEncoderPosition(resetAngle);
    }
  }

  public void disableRollerMotor(boolean shouldDisable) {
    isRollerMotorDisabled = shouldDisable;
    if (shouldDisable) rollerMotor.setRawPercentage(0);
  }

  public void disablePivotMotor(boolean shouldDisable) {
    isPivotMotorDisabled = shouldDisable;
    if (shouldDisable) rollerMotor.setRawPercentage(0);
  }
}
