package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import frc.helpers.CCSparkMax;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCSparkMax;
import frc.robot.subsystems.*;


public class SwerveDrive {
    
    public final SwerveModule frontRight =
    new SwerveModule(
        new CCSparkMax(
            "Front Right Drive",
            "frd",
            Constants.MotorConstants.FRONT_RIGHT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.FRONT_RIGHT_DRIVE_REVERSE,
            Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
            Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
        new CCSparkMax(
            "Front Right Turn",
            "frt",
            Constants.MotorConstants.FRONT_RIGHT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.FRONT_RIGHT_TURN_REVERSE,
            Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
            Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
        Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER,
        Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET,
        "Front Right");

public static final SwerveModule frontLeft =
    new SwerveModule(
        new CCSparkMax(
            "Front Left Drive",
            "fld",
            Constants.MotorConstants.FRONT_LEFT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.FRONT_LEFT_DRIVE_REVERSE,
            Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
            Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
        new CCSparkMax(
            "Front Left Turn",
            "flt",
            Constants.MotorConstants.FRONT_LEFT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.FRONT_LEFT_TURN_REVERSE,
            Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
            Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
        Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER,
        Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET,
        "Front Left");

public static final SwerveModule backRight =
    new SwerveModule(
        new CCSparkMax(
            "Back Right Drive",
            "brd",
            Constants.MotorConstants.BACK_RIGHT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.BACK_RIGHT_DRIVE_REVERSE,
            Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
            Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
        new CCSparkMax(
            "Back Right Turn",
            "brt",
            Constants.MotorConstants.BACK_RIGHT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.BACK_RIGHT_TURN_REVERSE,
            Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
            Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
        Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER,
        Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET,
        "Back Right");

public static final SwerveModule backLeft =
    new SwerveModule(
        new CCSparkMax(
            "Back Left Drive",
            "bld",
            Constants.MotorConstants.BACK_LEFT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.BACK_LEFT_DRIVE_REVERSE,
            Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
            Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
        new CCSparkMax(
            "Back Left Turn",
            "blt",
            Constants.MotorConstants.BACK_LEFT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.BACK_LEFT_TURN_REVERSE,
            Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
            Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
        Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER,
        Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET,
        "Back Left");

    SwerveModule [] swerveModules = {frontLeft, frontRight, backLeft, backRight};
    SwerveModuleState[] swerveModuleStates = {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    SwerveModulePosition[] swerveModulePositions;
    AHRS gyro = new AHRS(SPI.Port.kMXP);
    SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.DRIVE_KINEMATICS, new Rotation2d(gyro.getAngle()), swerveModulePositions, Constants.PoseConstants.hi);


    public void setSwerveModulePositions(){
        swerveModulePositions[0] = new SwerveModulePosition(0, new Rotation2d(frontLeft.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[1] = new SwerveModulePosition(0, new Rotation2d(frontRight.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[2] = new SwerveModulePosition(0, new Rotation2d(backLeft.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[3] = new SwerveModulePosition(0, new Rotation2d(backRight.getAbsoluteEncoderWithOffsetRotations()));
    }

    public void updateSwerveModulePositions(){
        swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[1] = new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[2] = new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[3] = new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getAbsoluteEncoderWithOffsetRotations()));
    }
    public void resetSwerveDrivePoseEstimator(){
        swerveDrivePoseEstimator.resetPosition(new Rotation2d(gyro.getAngle()), swerveModulePositions, new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getX(), swerveDrivePoseEstimator.getEstimatedPosition().getY(), gyro.getAngle()));
    }

    public void updateSwerveDrivePoseEstimator(){
        
    }

}
