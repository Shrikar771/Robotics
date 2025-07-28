package team5427.frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.InputStream;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5427.frc.robot.Constants;
import team5427.frc.robot.subsystems.intake.io.IntakeIO;
import team5427.frc.robot.subsystems.intake.io.IntakeIOInputAutoLogged;
import team5427.frc.robot.subsystems.intake.io.IntakeIOMagicTalonFX;
import team5427.lib.kinematics.shooter.projectiles.Linear;
import team5427.lib.motors.MagicSteelTalonFX;
import team5427.lib.motors.SteelTalonFX;

public class IntakeSubsystem extends SubsystemBase{
    private LinearVelocity intakingSpeed;
    private Rotation2d intakingAngle;
    private IntakeIO io;
    private IntakeIOInputAutoLogged inputAutoLogged;
    public Alert kIntakingSpeedOutofBounds = new Alert("OutOfBounds", "Intaking Speed Requested Out of Bounds", AlertType.kWarning);
    public Alert kIntakingAngleOutofBounds = new Alert("OutOfBounds", "Intaking Angle Requested Out of Bounds", AlertType.kWarning);
    public boolean gamePieceIntaked;


    public IntakeSubsystem(){
        inputAutoLogged = new IntakeIOInputAutoLogged();
        switch(Constants.currentMode){
            case REAL:
                io = new IntakeIOMagicTalonFX();
                break;
            case SIM:
                break;
            default:
                break;
        }
        intakingSpeed = MetersPerSecond.of(0.0);
        intakingAngle = Rotation2d.kZero;
        gamePieceIntaked = gamePieceIntaked();

    }
    @Override
    public void periodic(){
        io.updateInputs(inputAutoLogged);
        if(Math.abs(intakingSpeed.in(MetersPerSecond))>10.0){
            kIntakingSpeedOutofBounds.set(true);
        }
        else{
            kIntakingSpeedOutofBounds.set(false);
            io.setRollerSpeed(intakingSpeed);

        }
        if(intakingAngle.getDegrees()>IntakeConstants.kPivotMaxRotation.getDegrees()||intakingAngle.getDegrees()<IntakeConstants.kPivotMinimumRotation.getDegrees()){
            kIntakingAngleOutofBounds.set(true);
        }
        else{
            kIntakingAngleOutofBounds.set(false);
            io.setPivotRotation(intakingAngle);

        }
        Logger.processInputs("Intake/Inputs", inputAutoLogged);
        log();
    }

    public void log(){
        Logger.recordOutput("Intake/IntakingSpeed", intakingSpeed.in(MetersPerSecond));
        Logger.recordOutput("Intake/IntakingAngle", intakingAngle.getDegrees());
        Logger.recordOutput("Intake/GamePieceIntaked", gamePieceIntaked);

    }

    public void setIntakingSpeed(LinearVelocity speed){
        intakingSpeed = speed;
    }
    public void setIntakingRotation(Rotation2d angle){
        intakingAngle = angle;
    }
    public boolean gamePieceIntaked(){
        if(inputAutoLogged.rollerMotorCurrent.in(Amps)>=30){
            return true;
        }
        else{
            return false;
        }
               
    }
    public void resetPivotEncoderPosition(Rotation2d resetAngle){
        io.resetPivotEncoderPosition(resetAngle);
    }
    
    
    

    

}
