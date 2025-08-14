package team5427.frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.subsystems.intake.IntakeConstants;
import team5427.frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeStowed extends Command {
  private IntakeSubsystem subsystem;

  public IntakeStowed() {
    subsystem = IntakeSubsystem.getInstance();
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    subsystem.setIntakingAngle(IntakeConstants.kPivotIntakeRotation);
    subsystem.setIntakingSpeed(IntakeConstants.kRollerIntakeVelocity);
  }

  @Override
  public boolean isFinished() {
    return subsystem.intakeGamePiece();
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setIntakingAngle(IntakeConstants.kPivotStartingRotation);
    subsystem.setIntakingSpeed(IntakeConstants.kRollerStowedVelocity);
  }
}
