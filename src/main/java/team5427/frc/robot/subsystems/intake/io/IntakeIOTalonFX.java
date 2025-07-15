package team5427.frc.robot.subsystems.intake.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import team5427.lib.motors.SteelTalonFX;

public class IntakeIOTalonFX implements IntakeIO{
    private SteelTalonFX rollerMotor;
    private SteelTalonFX pivotMotor;

    public IntakeIOTalonFX(){

    }
    public void updateInputs(IntakeIOInput input){

    }

    public void setPivotRotation(Rotation2d rotation){}

    public void setRollerSpeed(LinearVelocity velocity){}

    public void setPivotRotation(Angle rotation){}

    public void setRolerSpeed(AngularVelocity velocity){}
    
    
}
