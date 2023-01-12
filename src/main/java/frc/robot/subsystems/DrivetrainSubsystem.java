package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;
import frc.robot.wrappers.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
    private final SwerveModule frontRight;
    private final SwerveModule frontLeft;
    private final SwerveModule backRight;
    private final SwerveModule backLeft;

    private final double frontRightDefault = 45;
    private final double frontLeftDefualt = -45;
    private final double backRightDefault = -45;
    private final double backLeftDefault = 45;

    public SwerveModuleState frontRightState = new SwerveModuleState(0, Rotation2d.fromDegrees(frontRightDefault));
    public SwerveModuleState frontLeftState = new SwerveModuleState(0, Rotation2d.fromDegrees(frontLeftDefualt));
    public SwerveModuleState backRightState = new SwerveModuleState(0, Rotation2d.fromDegrees(backRightDefault));
    public SwerveModuleState backLeftState = new SwerveModuleState(0, Rotation2d.fromDegrees(backLeftDefault));

    public ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    
    
    public DrivetrainSubsystem() {
        this.frontRight = new SwerveModule(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, 
                                        Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);
        this.frontLeft = new SwerveModule(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_MOTOR, 
                                        Constants.FRONT_LEFT_MODULE_STEER_ENCODER, Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
        this.backRight = new SwerveModule(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_MOTOR, 
                                        Constants.BACK_RIGHT_MODULE_STEER_ENCODER, Constants.BACK_RIGHT_MODULE_STEER_OFFSET);
        this.backLeft = new SwerveModule(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, Constants.BACK_LEFT_MODULE_STEER_MOTOR, 
                                        Constants.BACK_LEFT_MODULE_STEER_ENCODER, Constants.BACK_LEFT_MODULE_STEER_OFFSET);
    
        // frontRight is a problem child and wants to have its own PID values :(
        this.frontRight.setRotationPID(0.002, 0.000, 0.00001);
    }

    public void setFrontRight(SwerveModuleState state) {
        frontRightState = state;
    }
    public void setFrontLeft(SwerveModuleState state) {
        frontLeftState = state;
    }
    public void setBackRight(SwerveModuleState state) {
        backRightState = state;
    }
    public void setBackLeft(SwerveModuleState state) {
        backLeftState = state;
    }

    public void setAllToState(SwerveModuleState state) {
        setFrontRight(state);
        setFrontLeft(state);
        setBackRight(state);
        setBackLeft(state);
    }

    public void resetGyroscope() {
        m_gyro.reset();
    }

    public double getGyro() {
        return m_gyro.getAngle();
    } 

    public void periodic() {
        frontRight.setState(frontRightState);
        frontLeft.setState(frontLeftState);
        backRight.setState(backRightState);
        backLeft.setState(backLeftState);
    }
}
