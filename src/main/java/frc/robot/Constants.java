// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.51435;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.48895;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 51;  // -53 (10/26/22) // 51 (11/04/22)

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 156; // 23.8 (10/24/22) // 156 (11/04/22)

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 13; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 218; // -37 (10/26/2022) // 218 (11/04/22)

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 14;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 215; // -37 (10/31/22) // 215 (11/04/22)
}
