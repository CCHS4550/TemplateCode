

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.diagnostics.DoubleEntry;
import frc.helpers.CCSparkMax;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.maps.RobotMap;



//Subsystem that control the driving of the robot
public class DriveTrain extends SubsystemBase {
   
   //Initializing the Robot's Motors
    private final CCSparkMax frontLeftMotor = new CCSparkMax("Front Left", "fl", RobotMap.FRONT_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.FRONT_LEFT_REVERSE, RobotMap.DRIVE_ENCODER);
    private final CCSparkMax frontRightMotor = new CCSparkMax("Front Right", "fr", RobotMap.FRONT_RIGHT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.FRONT_RIGHT_REVERSE, RobotMap.DRIVE_ENCODER);
    private final CCSparkMax backLeftMotor = new CCSparkMax("Back Left", "bl", RobotMap.BACK_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_LEFT_REVERSE, RobotMap.DRIVE_ENCODER);
    private final CCSparkMax backRightMotor = new CCSparkMax("Back Right", "br", RobotMap.BACK_RIGHT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_RIGHT_REVERSE, RobotMap.DRIVE_ENCODER);

    //Group motors by left motors, right motors, and all motors
    MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
    MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);
    MotorControllerGroup allMotors = new MotorControllerGroup(leftMotors,rightMotors);

    //Call this function to change the power of all the motors to the maximum
    public void SetMotorsToMax(){
        allMotors.set(1.0);
    }
    //Call this function to turn off power to motors.
    public void turnOffAllMotors(){
        allMotors.set(0.0);
    }
    //Call this function with a powerLevel value (from -1.0 to 1.0) to change the power levels of all motors to the specified value
    public SetAllMotorsPower(Double powerLevel){
        if(powerLevel <= 1.0 && powerLevel >= -1.0){
            allMotors.set(powerLevel);
        }
        
    }
    //Call this function with a motorGroup (leftMotors, rightMotors, and allMotors) and a powerLevel value (from -1.0 to 1.0) to change the power levels of all motors in 1 group to the specified value
    public SetMotorGroup(MotorControllerGroup motorGroup, Double powerLevel){
        if(powerLevel <= 1.0 && powerLevel >= -1.0){
            motorGroup.set(powerLevel);
        }
        
    }
    //Call this function with a motor (frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor) and a powerLevel value (from 1.0 to 1.0) to change the power level of that 1 motor to the specified value.
    public SetMotorPower(CCSparkMax motor, Double powerLevel){
        if(powerLevel <= 1.0 && powerLevel >= -1.0){
            motor.set(powerLevel);
        }
    }



}
