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
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.585; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.59; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 1; // FIXME Set Pigeon ID

    public static final int DRIVER_CONTROLLER_X_AXIS_ID = 1; //Sideways
    public static final int DRIVER_CONTROLLER_Y_AXIS_ID = 2; //Forward
    public static final int DRIVER_CONTROLLER_Z_AXIS_ID = 3; //Flying jk its rotation

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 31; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 32; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 33; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(342.421875 +180); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 21; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 22; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 23; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(56.42578125); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 41; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 42; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 43; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(211.904296875 +180); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 12; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 13; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(271.7578125); // FIXME Measure and set back right steer offset

    //Shooter
    public static final int SHOOTER_TALONFX_MOTOR = 3;
    public static final int KICKER_WHEEL_TALONSRX_MOTOR = 4;
    public static final int SHOOTER_FLYWHEEL_RPM_LOW_GOAL = 2000;
    public static final int SHOOTER_FLYWHEEL_RPM_HIGH_GOAL = 5000;

    public static final double SWERVE_ROTATION_KP  = 0.2;
    public static final double SWERVE_ROTATION_KD = 0.1;
    public static final double MOVEMENT_FOLLOWING_KP = 1.5;
    public static final double MOVEMENT_FOLLOWING_KD = 0;
    public static final double ROTATION_FOLLOWING_KP = 6;
}
