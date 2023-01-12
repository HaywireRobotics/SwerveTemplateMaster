package frc.robot.wrappers;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    // In meters
    public static final double WHEEL_DIAMETER = 0.1016; // 4 inches

    private double ROTATION_KP = 0.003;
    private double ROTATION_KI = 0.00;  // 0.0015
    private double ROTATION_KD = 0.00001;
    private double ROTATION_KIZ = 0;
    private double ROTATION_KFF = 0;
    private double DRIVE_KP = 0.00007; //6.5e-5;
    private double DRIVE_KI = 0.0;  //5.5e-7;
    private double DRIVE_KD = 0.0; //0.001;
    private double DRIVE_KIZ = 0;
    private double DRIVE_KFF = 0;

    private final double OFFSET;

    private final PIDController rotationController = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
    private final PIDController driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);

    private SwerveModuleState desiredState;

    private final NEO rotationMotor;
    private final NEO driveMotor;

    private final CANCoder rotationEncoder;

    public SwerveModule(int driveID, int rotationID, int encoderID, double offset) {
        this(new NEO(driveID), new NEO(rotationID), new CANCoder(encoderID), offset);
    }

    public SwerveModule(NEO driveMotor, NEO rotationMotor, CANCoder rotationEncoder, double offset) {
        this.rotationMotor = rotationMotor;
        this.driveMotor = driveMotor;
        this.rotationEncoder = rotationEncoder;
        
        this.driveMotor.configurePIDFF(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KIZ, DRIVE_KFF);
        this.rotationMotor.configurePIDFF(ROTATION_KP, ROTATION_KI, ROTATION_KD, ROTATION_KIZ, ROTATION_KFF);

        this.rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        this.OFFSET = offset;
    }

    public void setRotationPID(double kp, double ki, double kd) {
        ROTATION_KP = kp;
        ROTATION_KI = ki;
        ROTATION_KD = kd;
        rotationController.setPID(kp, ki, kd);
    }

    public void setDrivePID(double kp, double ki, double kd) {
        DRIVE_KP = kp;
        DRIVE_KI = ki;
        DRIVE_KD = kd;
        driveController.setPID(kp, ki, kd);
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getRotation()));

        desiredState = state;
        // System.out.println(state.speedMetersPerSecond);

        // driveMotor.setVelocity(state.speedMetersPerSecond / (WHEEL_DIAMETER * Math.PI));
        // rotationMotor.setPosition(state.angle.getDegrees());

        double driveCalc = driveController.calculate(this.getSpeed(), state.speedMetersPerSecond / (WHEEL_DIAMETER * Math.PI));
        driveMotor.set(driveCalc);

        // System.out.println(driveCalc);

        double rotateCalc = rotationController.calculate(this.getRotation(), state.angle.getDegrees());
        // System.out.println(rotateCalc + " " + this.getRotation());
        rotationMotor.set(-rotateCalc);
    }

    public void driveDirect(double driveSpeed, double rotationSpeed) {
        driveMotor.set(driveSpeed);
        rotationMotor.set(rotationSpeed);
    }

    public double getSpeed() {
        return ((driveMotor.getVelocity() / 60) / 6.75) * (WHEEL_DIAMETER * Math.PI);
    }

    public double getRotation() {
        return rotationEncoder.getAbsolutePosition() - OFFSET;
    }

    // FIXME: idk what im doing
    public double getRotationMirrored() {
        double rotation = this.getRotation();
        if (rotation < 0) {
            rotation = 180 - rotation;
        }
        return rotation;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getRotation()));
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public int getID() {
        return rotationEncoder.getDeviceID();
    }
}
