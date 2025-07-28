package team5427.frc.robot.subsystems.intake.io;


import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class IntakeIOSim implements IntakeIO{
    private final IntakeSimulation intakeSimulation;
    public IntakeIOSim(Supplier<SwerveDriveSimulation> driveTrainSimulationSupplier){
        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        // Specify the type of game pieces that the intake can collect
        InfiniteRechargeBall.INFINITE_RECHARGABLE_BALL_INFO.type(),
        // Specify the drivetrain to which this intake is attached
        driveTrainSimulationSupplier.get(),
        // Width of the intake
        Meters.of(19.5),
        // The extension length of the intake beyond the robot's frame (when activated)
        Meters.of(0.2),
        // The intake is mounted on the back side of the chassis
        IntakeSimulation.IntakeSide.FRONT,
        // The intake can hold up to 1 note
        2);
    }

    public void updateInputs(IntakeIOInput input) {}

    public void setPivotRotation(Rotation2d rotation) {}

    public void setRollerSpeed(LinearVelocity velocity) {}

    public void setPivotRotation(Angle rotation) {}

    public void setRolerSpeed(AngularVelocity velocity) {}

    public void setRunning(boolean runIntake) {
        if (runIntake)
            intakeSimulation.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces
        else
            intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
    }
    public boolean isNoteInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }

}