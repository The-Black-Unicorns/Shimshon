// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int DEBUG_LEVEL = 1; // 1: last momants prints. 5:inside swerve stuff.
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.555; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.555; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 2; // FIXME Set Pigeon ID

    public static final int DRIVER_CONTROLLER_X_AXIS_ID = 1; // Sideways
    public static final int DRIVER_CONTROLLER_Y_AXIS_ID = 2; // Forward
    public static final int DRIVER_CONTROLLER_Z_AXIS_ID = 3; // Rotation

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 31;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 32;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 33;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(342.421875 + 180);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 41;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 42;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 43;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(211.904296875);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 13;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(271.7578125 + 90);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 21;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 22;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(56.42578125 + 90);

    // Shooter
    public static final int SHOOTER_TALONFX_MOTOR = 4;
    public static final int KICKER_WHEEL_TALONSRX_MOTOR = 7;
    public static final int CONVEYOR_TALONSRX_MOTOR = 7;
    public static final int INTAKE_TALONSRX_MOTOR = 8;
    public static final double KICKER_WHEEL_PERCENT = 0.2;
    public static final double CONVEYOR_SPEED_PERCENT_INTAKING = -1;
    public static final double CONVEYOR_SPEED_PERCENT_SHOOTING = -1;
    public static final double CONVEYOR_SPEED_PERCENT_REVERSE = 0.5;
    public static final int CONVEYER_REVERSE_DURATION_FRAMES = 30;
    public static final double INTAKE_SPEED_PERCENT = -1;
    public static final int SHOOTER_FLYWHEEL_RPM_LOW_GOAL = 3000;
    public static final int SHOOTER_FLYWHEEL_RPM_HIGH_GOAL = 1387;
    public static final int SHOOTER_FLYWHEEL_RPM_ERROR = 500;

    public static final boolean CARPET_COMPENSATION = true;
    public static final boolean INVERT_COMPENSATION = false;

    public static final double SWERVE_ROTATION_KP = 0.2;
    public static final double SWERVE_ROTATION_KD = 0.1;
    public static final double MOVEMENT_FOLLOWING_KP = 1.5;
    public static final double MOVEMENT_FOLLOWING_KD = 0;
    public static final double ROTATION_FOLLOWING_KP = 6;
    public static final double ROBOT_HOLD_ANGLE_KP = 3;
}
