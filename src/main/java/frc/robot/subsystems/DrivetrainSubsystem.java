// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    // Max theoretical velocity
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * SdsModuleConfigurations.MK4_L1.getDriveReduction()
            * SdsModuleConfigurations.MK4_L1.getWheelDiameter()
            * Math.PI;

    // Max theoretical angular velocity
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    

    // the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);

    //Modules
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private SwerveDriveOdometry driveOdometry;

    public Pose2d robotPose = new Pose2d();

    private boolean extraBrake = false;

    public double holdAngleSetpoint = Math.toRadians(0);
    int fromRotationCounter = 0;
    boolean resetEncoder = false;
    private PIDController holdRobotAngleController = new PIDController(Constants.ROBOT_HOLD_ANGLE_KP, 0, 0);

    private boolean enabled = false;

    private int framesSinceEnable = 0;

    private Rotation2d gyroAngle;
    private Rotation2d gyroOffset = new Rotation2d();
    private boolean compensationDirection;

    public DrivetrainSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0,
                        0),
                Mk4SwerveModuleHelper.GearRatio.L1, FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET);
        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2,
                        0),
                Mk4SwerveModuleHelper.GearRatio.L1, FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);
        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left M      odule", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L1, BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);
        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6,
                        0),
                Mk4SwerveModuleHelper.GearRatio.L1, BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);

        updateGyroAngle();

        driveOdometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation());

        // SET PID
        double steerkP = 0.4;
        double steerkI = 0;
        double steerkD = 0.3;
        double steerKf = 0;
        updateFalconPID(12, steerkP, steerkI, steerkD, steerKf, NeutralMode.Brake);
        updateFalconPID(22, steerkP, steerkI, steerkD, steerKf, NeutralMode.Brake);
        updateFalconPID(32, steerkP, steerkI, steerkD, steerKf, NeutralMode.Brake);
        updateFalconPID(42, steerkP, steerkI, steerkD, steerKf, NeutralMode.Brake);
        double drivekP = 0.15;
        double drivekI = 0;
        double drivekD = 0.05;
        double drivekF = 0.05;
        updateFalconPID(11, drivekP, drivekI, drivekD, drivekF, NeutralMode.Brake);
        updateFalconPID(21, drivekP, drivekI, drivekD, drivekF, NeutralMode.Brake);
        updateFalconPID(31, drivekP, drivekI, drivekD, drivekF, NeutralMode.Brake);
        updateFalconPID(41, drivekP, drivekI, drivekD, drivekF, NeutralMode.Brake);

        // Pigeon status frames
        m_pigeon.setStatusFramePeriod(6, 10);
        m_pigeon.setStatusFramePeriod(11, 10);
        m_pigeon.setStatusFramePeriod(4, 10);

        holdRobotAngleController.disableContinuousInput();
        holdRobotAngleController.setTolerance(Math.toRadians(2));


        //Get carpet compensation direction
        if (DriverStation.getAlliance() == Alliance.Red) {
            compensationDirection = true;
        } else {
            compensationDirection = false;
        }
        compensationDirection = compensationDirection ^ Constants.INVERT_COMPENSATION;
    }

    public void zeroPosition(Pose2d newPose) {
        System.out.println("Zero!");

        gyroOffset = Rotation2d.fromDegrees(getGyroRotationRaw().getDegrees() - newPose.getRotation().getDegrees());
        holdAngleSetpoint = newPose.getRotation().getRadians();

        driveOdometry.resetPosition(newPose, newPose.getRotation());
    }

    public void matchEncoders() {
        resetEncoder = true;
    }

    private void resetHoldAngle() {
        holdAngleSetpoint = getGyroscopeRotation().getRadians();
        fromRotationCounter = 0;
    }

    public void updateGyroAngle() {
        gyroAngle = Rotation2d.fromDegrees(m_pigeon.getFusedHeading() * 1.00278552 - gyroOffset.getDegrees());
        // System.out.println(gyroAngle.getDegrees() + ", " + gyroOffset.getDegrees());
        // System.out.println(callsPerLoop);
        // callsPerLoop = 0;
    }

    public Rotation2d getGyroscopeRotation() {
        // return Rotation2d.fromDegrees(0);
        // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading() * 1.00278552);
        // callsPerLoop++;
        return gyroAngle;
        // if (m_navx.isMagnetometerCalibrated()) {
        // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        // }
        // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    private Rotation2d getGyroRotationRaw() {
        return Rotation2d.fromDegrees(m_pigeon.getFusedHeading() * 1.00278552);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        if (chassisSpeeds.omegaRadiansPerSecond == 0) {
            fromRotationCounter++;
            if (chassisSpeeds.vxMetersPerSecond != 0 || chassisSpeeds.vyMetersPerSecond != 0) {
                if (fromRotationCounter >= 25)
                    chassisSpeeds.omegaRadiansPerSecond = holdRobotAngleController
                            .calculate(getGyroscopeRotation().getRadians(), holdAngleSetpoint);
            } else {
                // resetHoldAngle();
            }

        } else {
            resetHoldAngle();
        }

        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(m_chassisSpeeds, new Translation2d(0, 0));
        driveWithModuleStates(states);

        if (framesSinceEnable < 750)
            updateOdometry();
        if (enabled)
            framesSinceEnable++;

        // System.out.println("Pose is: " + robotPose.getX() + ", " + robotPose.getY()
        // +", " + robotPose.getRotation().getDegrees() + ", " +
        // getGyroscopeRotation().getDegrees());

        resetEncoder = false;
    }

    private void driveWithModuleStates(SwerveModuleState[] states) {
        if (m_chassisSpeeds.vxMetersPerSecond != 0 || m_chassisSpeeds.vyMetersPerSecond != 0
                || m_chassisSpeeds.omegaRadiansPerSecond != 0) {
            frontLeftModule.set(
                    states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                    states[0].angle.getRadians(), resetEncoder);
            frontRightModule.set(
                    states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                    states[1].angle.getRadians(), resetEncoder);
            backLeftModule.set(
                    states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                    states[2].angle.getRadians(), resetEncoder);
            backRightModule.set(
                    states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                    states[3].angle.getRadians(), resetEncoder);
        } else if (extraBrake) {
            frontLeftModule.set(0, Math.toRadians(45), resetEncoder);
            frontRightModule.set(0, Math.toRadians(135), resetEncoder);
            backLeftModule.set(0, Math.toRadians(135), resetEncoder);
            backRightModule.set(0, Math.toRadians(45), resetEncoder);
        } else {
            frontLeftModule.set(0, frontLeftModule.getSteerAngle(), resetEncoder);
            frontRightModule.set(0, frontRightModule.getSteerAngle(), resetEncoder);
            backLeftModule.set(0, backLeftModule.getSteerAngle(), resetEncoder);
            backRightModule.set(0, backRightModule.getSteerAngle(), resetEncoder);
        }
    }

    private void updateOdometry() {

        // Updating the odometry
        Pose2d tempPose = driveOdometry.update(getGyroscopeRotation(),
                new SwerveModuleState(frontLeftModule.getDriveVelocity(),
                        new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModuleState(frontRightModule.getDriveVelocity(),
                        new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModuleState(backLeftModule.getDriveVelocity(),
                        new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModuleState(backRightModule.getDriveVelocity(),
                        new Rotation2d(backRightModule.getSteerAngle())));

        // Compensation for field carpet grain
        if (Constants.CARPET_COMPENSATION) {
            double difference = tempPose.getX() - robotPose.getX();
            if (difference > 0 && compensationDirection) {
                difference = difference * (1 - (double) 100 / 105);

                Pose2d tempPose2 = tempPose
                        .plus(new Transform2d(new Translation2d(-difference, 0),
                                new Rotation2d()));
                tempPose = tempPose2;
            } else if (difference < 0 && !compensationDirection) {
                difference = difference * (1 - (double) 100 / 105);

                Pose2d tempPose2 = tempPose
                        .plus(new Transform2d(new Translation2d(difference, 0),
                                new Rotation2d()));
                tempPose = tempPose2;
            }
            driveOdometry.resetPosition(tempPose, getGyroscopeRotation());
        }
        robotPose = tempPose;
    }

    public void onEnable() {
        holdAngleSetpoint = getGyroscopeRotation().getRadians();
        enabled = true;
        framesSinceEnable = 0;
    }

    public void onDisable() {
        enabled = false;
    }

    public void enableExtraBrake() {
        extraBrake = true;
    }

    public void disableExtraBrake() {
        extraBrake = false;
    }

    public static void updateFalconPID(int talonCanID, double kP, double kI, double kD, double kF,
            NeutralMode neutralMode) {
        TalonFXConfiguration talonConfiguration = new TalonFXConfiguration();
        talonConfiguration.slot0.kP = kP;
        talonConfiguration.slot0.kI = kI;
        talonConfiguration.slot0.kD = kD;
        talonConfiguration.slot0.kF = kF;
        // System.out.println("kP = " + kP + ", kD = " + kD);
        // Shuffleboard.getTab("Drivetrain").add("D",kD);
        TalonFX talon = new TalonFX(talonCanID);
        talon.configAllSettings(talonConfiguration);
        talon.setNeutralMode(neutralMode);
        talon.setStatusFramePeriod(1, 20);
    }

    public Pose2d getVisionPose(){
        
        double distance = getDistanceMetersVision();

        
        return new Pose2d();
    }

    public double getDistanceMetersVision () {

        //In degrees
        double cameraAngle = 20;
        double targetY =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0) + cameraAngle;
        //Between The camera to the target
        double deltaY = 1.2;

        double distance = deltaY / Math.sin(Math.toRadians(cameraAngle + targetY));
        return distance;
    }

}
