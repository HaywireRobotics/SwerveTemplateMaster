package frc.robot.commands;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;



public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_subsystem;
    // private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    private final XboxController controller;
    // private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);


    // new Button(m_controller::getBackButton).whenPressed(DefaultDriveCommand::resetGyroscope);

    // public void resetGyroscope() {
    //     m_gyro.reset();
    // }

    //the buffer not only affects the "deadzone", 
    // but also prohibits small angles near the x/y axis
    private static final double buffer = 0.05; 

    private static final double maxSpeed = 1000;

    public DefaultDriveCommand(DrivetrainSubsystem subsystem, XboxController xboxController) {
        this.m_subsystem = subsystem;
        this.controller = xboxController;

        addRequirements(subsystem);

        
    }

    private double applyBuffer(double x) {
        if ((x < buffer && x > 0) || (x > buffer && x < 0)) {
            x = 0;
        }
        // double scaled = (x - buffer) / (1 - buffer);
        // return scaled * scaled;
        return x;
    }

    @Override
    public void execute() {
        double rightX = controller.getRightX();
        double leftX = controller.getLeftX();
        double leftY = controller.getLeftY();

        rightX = -applyBuffer(rightX);
        leftX = applyBuffer(leftX);
        leftY = -applyBuffer(leftY);

        System.out.println(m_subsystem.getGyro());

        double driveAngle = 0;
        if (leftX != 0 && leftY != 0) {
            driveAngle = Math.toDegrees(Math.atan(leftX / leftY));
            // adjusts negative angles to be in range of 0 to 360
            if (driveAngle < 0) {
                driveAngle += 360;
            }
        // defaults to axis lines to avoid NaN errors with Math.atan
        } else if (leftX == 0) {
            driveAngle = 0;
        } else if (leftY == 0) {
            driveAngle = 90;
        }

        // make directions feild-centric
        driveAngle -= m_subsystem.getGyro();

        // accounts for Math.hypot only returning positive values
        double driveSpeed = 0.0;
        if (leftX > 0 && leftY >= 0) {
            driveSpeed = maxSpeed * Math.abs(Math.hypot(leftX, leftY));
        } else if (leftY <= 0) {
            driveSpeed = -maxSpeed * Math.abs(Math.hypot(leftX, leftY));
        } else {
            driveSpeed = maxSpeed * Math.abs(Math.hypot(leftX, leftY));
        }

        SwerveModuleState frontLeftDrive = new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(driveAngle));
        SwerveModuleState frontRightDrive = new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(driveAngle));
        SwerveModuleState backLeftDrive = new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(driveAngle));
        SwerveModuleState backRightDrive = new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(driveAngle));
        
        double rotateSpeed = maxSpeed * rightX;

        // different signs accounts for orientation of modules
        SwerveModuleState frontLeftRotate = new SwerveModuleState(-rotateSpeed, Rotation2d.fromDegrees(45));
        SwerveModuleState frontRightRotate = new SwerveModuleState(rotateSpeed, Rotation2d.fromDegrees(-45));
        SwerveModuleState backLeftRotate = new SwerveModuleState(-rotateSpeed, Rotation2d.fromDegrees(-45));
        SwerveModuleState backRightRotate = new SwerveModuleState(rotateSpeed, Rotation2d.fromDegrees(45));

        m_subsystem.setFrontLeft(this.addStates(frontLeftDrive, frontLeftRotate));
        m_subsystem.setFrontRight(this.addStates(frontRightDrive, frontRightRotate));
        m_subsystem.setBackLeft(this.addStates(backLeftDrive, backLeftRotate));
        m_subsystem.setBackRight(this.addStates(backRightDrive, backRightRotate));
    }

    private SwerveModuleState addStates(SwerveModuleState a, SwerveModuleState b) {
        // here we treat ServeModuleState's as vectors and add their component forms 
        // the cart is short for cartesian bc why not
        // [0] is x and [1] is y
        double[] a_cart = {a.speedMetersPerSecond*a.angle.getCos(), a.speedMetersPerSecond*a.angle.getSin()};
        double[] b_cart = {b.speedMetersPerSecond*b.angle.getCos(), b.speedMetersPerSecond*b.angle.getSin()};
        double[] out_cart = {a_cart[0] + b_cart[0], a_cart[1] + b_cart[1]};
        
        double speed = Math.hypot(out_cart[0], out_cart[1]);
        if (out_cart[0] < 0) speed *= -1; // this is needed bc Math.atan() only returns -90 to 90, so for negative x it needs to flip
        double angle = Math.toDegrees(Math.atan(out_cart[1] / out_cart[0]));
        SwerveModuleState out = new SwerveModuleState(speed, Rotation2d.fromDegrees(angle));

        return out;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setAllToState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
