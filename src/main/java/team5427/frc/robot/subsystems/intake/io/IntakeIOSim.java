package team5427.frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.function.Supplier;
import org.ironmaple.simulation.*;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import team5427.frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

  private final IntakeSimulation intakeSimulation;
  // 2d vector
  // accepting 1d vector as input
  // Outputting 2d vector
  private LinearSystem<N2, N1, N2> pivotMotor;
  private LinearSystem<N1, N1, N1> rollerMotor;

  private ProfiledPIDController pivotMProfiledPIDController;
  private PIDController rollerMotorPIDController;

  private DCMotorSim pivotMotorSim;
  private FlywheelSim rollerMotorSim;

  public IntakeIOSim(Supplier<SwerveDriveSimulation> driveTrainSimulationSupplier) {
    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            InfiniteRechargeBall.INFINITE_RECHARGABLE_BALL_INFO.type(),
            driveTrainSimulationSupplier.get(),
            Meters.of(0.7),
            Meters.of(0.2),
            IntakeSimulation.IntakeSide.FRONT,
            2);
    // pivotMotor = new DCMotorSim(null,IntakeConstants.kPivotMotorConfiguration.withFOC ?
    // DCMotor.getKrakenX60Foc(1): DCMotor.getKrakenX60(1 ));
    pivotMotor =
        LinearSystemId.createSingleJointedArmSystem(
            IntakeConstants.kPivotMotorConfiguration.withFOC
                ? DCMotor.getKrakenX60Foc(1)
                : DCMotor.getKrakenX60(1),
            0.01,
            IntakeConstants.kPivotMotorConfiguration.gearRatio.getMathematicalGearRatio());

    rollerMotor =
        LinearSystemId.createFlywheelSystem(
            IntakeConstants.kRollerMotorConfiguration.withFOC
                ? DCMotor.getKrakenX60Foc(1)
                : DCMotor.getKrakenX60(1),
            0.0001,
            IntakeConstants.kRollerMotorConfiguration.gearRatio.getMathematicalGearRatio());

    pivotMProfiledPIDController =
        new ProfiledPIDController(
            IntakeConstants.kPivotMotorSimulatedkP,
            0,
            0,
            new Constraints(
                IntakeConstants.kPivotMotorConfiguration.maxVelocity,
                IntakeConstants.kPivotMotorConfiguration.maxAcceleration));

    rollerMotorPIDController = new PIDController(IntakeConstants.kRollerMotorSimulatedkP, 0, 0);

    pivotMotorSim = new DCMotorSim(pivotMotor, DCMotor.getKrakenX60Foc(1));
    rollerMotorSim = new FlywheelSim(rollerMotor, DCMotor.getKrakenX60Foc(1));
  }

  public void updateInputs(IntakeIOInput inputs) {
    inputs.rollerMotorConnected = true;
    inputs.pivotMotorConnected = true;

    inputs.pivotMotorAngularAcceleration = pivotMotorSim.getAngularAcceleration();
    inputs.pivotMotorAngularVelocity = pivotMotorSim.getAngularVelocity();
    inputs.pivotMotorCurrent = Amps.of(pivotMotorSim.getCurrentDrawAmps());
    inputs.pivotMotorRotation =
        Rotation2d.fromRotations(pivotMotorSim.getAngularPositionRotations());
    inputs.pivotMotorTemperature = Celsius.of(30);
    inputs.pivotMotorVoltage = Volts.of(pivotMotorSim.getInputVoltage());

    inputs.rollerMotorAngularAcceleration = rollerMotorSim.getAngularAcceleration();
    inputs.rollerMotorAngularVelocity = rollerMotorSim.getAngularVelocity();
    inputs.rollerMotorCurrent = Amps.of(rollerMotorSim.getCurrentDrawAmps());
    inputs.rollerMotorLinearAcceleration =
        MetersPerSecondPerSecond.of(
            inputs.rollerMotorAngularAcceleration.in(RotationsPerSecondPerSecond)
                * (IntakeConstants.kRollerMotorConfiguration.finalDiameterMeters * Math.PI));
    inputs.rollerMotorLinearVelocity =
        MetersPerSecond.of(
            inputs.rollerMotorAngularVelocity.in(RadiansPerSecond)
                * (IntakeConstants.kRollerMotorConfiguration.finalDiameterMeters * Math.PI));

    inputs.rollerMotorTemperature = Celsius.of(30);
    inputs.rollerMotorVoltage = Volts.of(rollerMotorSim.getInputVoltage());
  }

  public void setPivotRotation(Rotation2d rotation) {
    double inputVoltage =
        pivotMProfiledPIDController.calculate(
            pivotMotorSim.getAngularPositionRotations()
                * IntakeConstants.kPivotGearRatio.getMathematicalGearRatio(),
            rotation.getRotations());
    pivotMotorSim.setInputVoltage(inputVoltage);
  }

  public void setPivotRotation(Angle rotation) {
    double inputVoltage =
        pivotMProfiledPIDController.calculate(
            pivotMotorSim.getAngularPositionRotations()
                * IntakeConstants.kPivotGearRatio.getMathematicalGearRatio(),
            rotation.in(Rotations));
    pivotMotorSim.setInputVoltage(inputVoltage);
  }

  public void setRollerSpeed(LinearVelocity velocity) {
    double rotationsPerSecond =
        velocity.in(MetersPerSecond)
            / IntakeConstants.kPivotMotorConfiguration.finalDiameterMeters
            * Math.PI;
    double inputVoltage =
        rollerMotorPIDController.calculate(
            rollerMotorSim.getAngularVelocity().in(RotationsPerSecond)
                * IntakeConstants.kRollerGearRatio.getMathematicalGearRatio(),
            rotationsPerSecond * IntakeConstants.kRollerGearRatio.getMathematicalGearRatio());
    rollerMotorSim.setInputVoltage(inputVoltage);
  }

  public void setRollerSpeed(AngularVelocity velocity) {

    double inputVoltage =
        rollerMotorPIDController.calculate(
            rollerMotorSim.getAngularVelocity().in(RotationsPerSecond)
                * IntakeConstants.kRollerGearRatio.getMathematicalGearRatio(),
            velocity.in(RotationsPerSecond)
                * IntakeConstants.kRollerGearRatio.getMathematicalGearRatio());
    rollerMotorSim.setInputVoltage(inputVoltage);
  }

  public void setRunning(boolean runIntake) {
    if (runIntake)
      intakeSimulation
          .startIntake(); // Extends the intake out from the chassis frame and starts detecting
    // contacts with game pieces
    else
      intakeSimulation
          .stopIntake(); // Retracts the intake into the chassis frame, disabling game piece
    // collection
  }

  public boolean isBallInsideIntake() {
    return intakeSimulation.getGamePiecesAmount()
        != 0; // True if there is a game piece in the intake
  }
}
