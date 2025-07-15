// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5427.frc.robot;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.drivers.ComplexGearRatio;
import team5427.lib.motors.MotorConfiguration;
import team5427.lib.motors.MotorUtil;
import team5427.lib.motors.MotorConfiguration.IdleState;
import team5427.lib.motors.MotorConfiguration.MotorMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String kCanivoreBusName = "canivore_bus_name";
  public static final double kOdometryFrequency =
      new CANBus(Constants.kCanivoreBusName).isNetworkFD() ? 250.0 : 100.0;

  public static Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double kLoopSpeed = Units.millisecondsToSeconds(20);

  public static final boolean kIsTuningMode = false;

  public static RobotConfig config;
  public static final CANDeviceId kFlyWheelMotorCanId = new CANDeviceId(4);
  //Create new Can Device ID for flywheel motor
  public static MotorConfiguration kFlyWheelMotorConfiguration = new MotorConfiguration();
  // Create a new motor configuration for flywheel motor

  static{
    //Define motor flywheel configuration gear ratio. 
    // What this defines is that a 2 tooth gear (driven) is connected to a 6 tooth gear (driving)
    //and that 6 tooth gear is connected to a 10 tooth gear with a 4 tooth driven gear.
    kFlyWheelMotorConfiguration.gearRatio = new ComplexGearRatio((2.0/6.0), (4.0/10.0));
     //Sets the idle state of the motor
    kFlyWheelMotorConfiguration.idleState = IdleState.kBrake;
   //the mode of the motor
    kFlyWheelMotorConfiguration.mode = MotorMode.kFlywheel;
    //Includes FOC (all the time)
    kFlyWheelMotorConfiguration.withFOC = true;
    //Sets standard maximum velocity to the standard maximum velocity of a KrakenX60 with FOC
    kFlyWheelMotorConfiguration.maxVelocity = kFlyWheelMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenX60FOC_MaxRPM);
    //maximum acceleration is equal to velocity multiplied by a reasonable constant, in this case 2

    kFlyWheelMotorConfiguration.maxAcceleration = kFlyWheelMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenX60FOC_MaxRPM)*2.0;
    
    //Sets all the necessary attributes of the motor configuration to standard values
    kFlyWheelMotorConfiguration.currentLimit = 40;
    //Acceleration feed forward
    kFlyWheelMotorConfiguration.kA = 1.0;
    //Derivative term part of PID
    kFlyWheelMotorConfiguration.kD = 1.0;
    //arbitrary feed forward
    kFlyWheelMotorConfiguration.kFF = 1.0;
    //Porportional term of the PID
    kFlyWheelMotorConfiguration.kP = 1.0;
    //Gravitation feed forward
    kFlyWheelMotorConfiguration.kG = 1.0;
    //Static friction feed forward
    kFlyWheelMotorConfiguration.kS = 1.0;
    //ki = integral term of PID
  }
  


  public static class DriverConstants {
    public static final int kDriverJoystickPort = 0;
    public static final int kOperatorJoystickPort = 1;
    public static final double kDriverControllerJoystickDeadzone = 0.0;
  }
}
