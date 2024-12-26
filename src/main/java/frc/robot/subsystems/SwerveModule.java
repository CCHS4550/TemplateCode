

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
public class SwerveModule extends SubsystemBase{
    private CCSparkMax driveMotor;
    private CCSparkMax turnMotor;
    private PIDController drivePidController, turnPidController;
    private SimpleMotorFeedforward driveFeedForward;
    private AnalogEncoder absoluteEncoder;
    private int absoluteEncoderPort;
    private double absoluteEncoderOffset;
    private String name;

    public SwerveModule (CCSparkMax turnMotor, CCSparkMax driveMotor,int absoluteEncoderPort, double absoluteEncoderOffset, String name){
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        absoluteEncoder = new AnalogEncoder(absoluteEncoderPort);
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        turnPidController = new PIDController(0.75, 0, 0); //obviously test and change these
        drivePidController = new PIDController(1,0,0);
        turnPidController.enableContinuousInput(0, 2*Math.PI);

        driveFeedForward = new SimpleMotorFeedforward(.09, .09, .09); //change these too\

        absoluteEncoder.setDistancePerRotation(2*Math.PI);
        
        this.name = name;
        
    }

    public String getName(){
        return name;
    }

    public double getAbsoluteEncoderWithOffsetRadians(){
       return Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition()+absoluteEncoderOffset);
    }

    public double getAbsoluteEncoderWithOffsetRotations(){
        return absoluteEncoder.getAbsolutePosition()+absoluteEncoderOffset;
    }

    public double getAbsoluteEncoderWithoutOffsetRadians(){
        return Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition());
    }

    public void resetEncoders(){
        driveMotor.reset();
        turnMotor.setPosition(getAbsoluteEncoderWithOffsetRotations());
    }

    public double getDrivePosition(){
        return driveMotor.getPosition();
    }
    
    public double getTurnPosition(){
        return turnMotor.getPosition();
    }
    public double getDriveVelocity(){
        return driveMotor.getVelocity();
    }
    public double getTurnVelocity(){
        return turnMotor.getVelocity();
    }
    
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }
    public void turnTo(SwerveModuleState desiredState){
        double desiredAngle = desiredState.angle.getRotations();
        turnMotor.setVelocityConversionFactor(0.98);//obviously check this
        turnMotor.setVoltageFromSpeed(turnPidController.calculate(getAbsoluteEncoderWithOffsetRotations(),desiredAngle));
    }

    public void driveTo (SwerveModuleState desiredState){
        double desiredVelocity = desiredState.speedMetersPerSecond;
        double driveFF = driveFeedForward.calculate(getDriveVelocity(), desiredVelocity);
        double drivePID = drivePidController.calculate(getDriveVelocity(),desiredVelocity);

        driveMotor.setVoltage(driveFF+drivePID);
    }

    public void setState(SwerveModuleState desiredState){
        if (Math.abs(desiredState.speedMetersPerSecond) <= .005) {
            stop();
            return;
          }
      
          Rotation2d encoderRotation = new Rotation2d(getState().angle.getRadians());
      
          SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
          // Minimizes side drift when driving
          state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();
      
          driveTo(desiredState);
        //   Logger.recordOutput("desiredState - Meters per Second", state.speedMetersPerSecond);
          turnTo(desiredState);
          // setTurnPosition();
      
    }

    public void stop(){
        driveMotor.setVoltage(0);
        turnMotor.setVoltage(0);
    }

    public void manualSetSpeed(double driveSpeed, double turnSpeed){
        driveMotor.set(driveSpeed);
        turnMotor.set(turnSpeed);
    }
    public void manualSetSpeedToVoltage(double driveSpeed, double turnSpeed){
        driveMotor.setVoltageFromSpeed(driveSpeed);
        turnMotor.setVoltageFromSpeed(turnSpeed);

    }
    public void manualSetVoltage(double driveVoltage, double turnVoltage){
        driveMotor.setVoltage(driveVoltage);
        turnMotor.setVoltage(turnVoltage);
    }
    


}
