// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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

    public static final String MAIN_DASHBOARD_TAB_NAME = "Robot";

    public static final int DEBUG_LEVEL = 1; // 1: last momants prints. 5:inside swerve stuff.
    //Left right distance
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.555;
    //Front to back distance
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.555;

    public static final int PIGEON_ID = 2;


    public static final int DRIVER_CONTROLLER_Y_AXIS_ID = 1; // Sideways
    public static final int DRIVER_CONTROLLER_X_AXIS_ID = 2; // Forward
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
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(261.7218017578125);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 21;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 22;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(56.42578125 + 90);


    //Wheels at 45 degrees while stationary
    public static final boolean EXTRA_BRAKE = false;
    //Use PS4 controller on port 2 instead of QX7 on port 0
    public static final boolean ALTERNATE_DRIVE = true;
    //Use PID to keep the robot straight while not rotating (Works great, cancel only with premission from Ido or Yonatan)
    public static final boolean HOLD_ANGLE = true;

//talon
    //Shooter
    //ID
        public static final int SHOOTER_TALONFX_ID = 4;
        public static final int KICKER_WHEEL_TALONSRX_ID = 7;
        public static final int CONVEYOR_TALONSRX_ID = 7;
        public static final int INTAKE_TALONSRX_ID = 8;
    //Speeds
        public static final double KICKER_WHEEL_PERCENT = 0.2;
        public static final double CONVEYOR_SPEED_PERCENT_INTAKING = -1;
        public static final double CONVEYOR_SPEED_PERCENT_SHOOTING = -1;
        public static final double CONVEYOR_SPEED_PERCENT_REVERSE = 1;
        public static final double INTAKE_SPEED_PERCENT = -1;
        //Change to -0.2 - -0.3 if balls escape from shooter
        public static final double REVERSE_FALCON_SPEED = 0;
        //RPM - replace the two options to switch between High and Low 
        public static final int SHOOTER_FLYWHEEL_RPM_LOW_GOAL = 2500;
        public static final int SHOOTER_FLYWHEEL_RPM_HIGH_GOAL = 1400;
        public static final int SHOOTER_FLYWHEEL_RPM_ERROR = 500;
    
    //Delays
        public static final int CONVEYER_REVERSE_DURATION_FRAMES = 15;



        
    //Odometry
    public static final boolean CARPET_COMPENSATION = true;
    public static final boolean INVERT_COMPENSATION = false;

    public static final double SWERVE_ROTATION_KP = 0.2;
    public static final double SWERVE_ROTATION_KD = 0.1;
    public static final double MOVEMENT_FOLLOWING_KP = 1.5;
    public static final double MOVEMENT_FOLLOWING_KD = 0;
    public static final double ROTATION_FOLLOWING_KP = 6;
    public static final double ROBOT_HOLD_ANGLE_KP = 3;

    //Limelight
    public static final double LIMELIGHT_ANGLE_DEG = 20;
    public static final double LIMELIGHT_TO_GOAL_Y_DELTA_METER = 1.2;
    public static final Translation2d GOAL_LOCATION = new Translation2d(0, 0);
}
