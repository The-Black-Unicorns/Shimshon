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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * SdsModuleConfigurations.MK4_L1.getDriveReduction()
            * SdsModuleConfigurations.MK4_L1.getWheelDiameter()
            * Math.PI;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     * 
     * // Here we calculate the theoretical maximum angular velocity. You can also
     * // replace this with a measured amount.
     **/
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DRIVETRAIN_WHEELBASE_METERS / 2.0));;

    // By default we use a Pigeon for our gyroscope. But if you use another
    // gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating
    // the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
    // private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX
    // connected over MXP

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private SwerveDriveOdometry driveOdometry;

    public Pose2d robotPose = new Pose2d();

    public boolean extraBrake = false;

    double angle = 0;
    int count = 0;
    double previousSpeed = 0;
    public double holdAngleSetpoint = Math.toRadians(0);
    int fromRotationCounter = 0;
    boolean resetEncoder = false;

    private PIDController holdRobotAngleController = new PIDController(Constants.ROBOT_HOLD_ANGLE_KP, 0, 0);

    private Rotation2d gyroAngle;

    private int callsPerLoop = 0;

    public DrivetrainSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of
                // the module on the dashboard.

                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0,
                        0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4SwerveModuleHelper.GearRatio.L1,
                // This is the ID of the drive motor
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                // This is the ID of the steer motor
                FRONT_LEFT_MODULE_STEER_MOTOR,
                // This is the ID of the steer encoder
                FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case,
                // zero is facing straight forward)
                FRONT_LEFT_MODULE_STEER_OFFSET);

        // We will do the same for the other modules
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
    }

    public void zeroPosition() {
        System.out.println("Zero!");
        m_pigeon.setFusedHeading(0.0);
        holdAngleSetpoint = Math.toRadians(0);
        // m_navx.zeroYaw();

        // Reset odometry angle
        driveOdometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), getGyroscopeRotation());
    }

    public void zeroPosition(Pose2d newPose) {
        System.out.println("Zero!");
        m_pigeon.setFusedHeading(newPose.getRotation().getDegrees());
        holdAngleSetpoint = newPose.getRotation().getRadians();
        // m_navx.zeroYaw();

        // Reset odometry angle
        driveOdometry.resetPosition(newPose, getGyroscopeRotation());
    }

    public void matchEncoders() {
        resetEncoder = true;
    }

    public void updateGyroAngle() {
        gyroAngle = Rotation2d.fromDegrees(m_pigeon.getFusedHeading() * 1.00278552);
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

    public void drive(ChassisSpeeds chassisSpeeds) {
        if (chassisSpeeds.omegaRadiansPerSecond == 0) {
            fromRotationCounter++;
            if ((chassisSpeeds.vxMetersPerSecond != 0 || chassisSpeeds.vyMetersPerSecond != 0)
                    && fromRotationCounter >= 25) {
                chassisSpeeds.omegaRadiansPerSecond = holdRobotAngleController
                        .calculate(getGyroscopeRotation().getRadians(), holdAngleSetpoint);
            }

        } else {
            fromRotationCounter = 0;
            holdAngleSetpoint = getGyroscopeRotation().getRadians();
        }

        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(m_chassisSpeeds, new Translation2d(0, 0));
        // SwerveDriveKinematics.normalizeWheelSpeeds(states,
        // MAX_VELOCITY_METERS_PER_SECOND);
        driveWithModuleStates(states);
        updateOdometry();

        // SmartDashboard.putNumber("Wheel sent speed", states[0].speedMetersPerSecond);
        // SmartDashboard.putNumber("Wheel sent Voltage", states[0].speedMetersPerSecond
        // / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE);
        // SmartDashboard.putNumber("Wheel measured speed",
        // frontLeftModule.getDriveVelocity());
        // System.out.println("Pose is: " + robotPose.getX() + ", " + robotPose.getY()
        // +", " + robotPose.getRotation().getDegrees() + ", " +
        // getGyroscopeRotation().getDegrees()%360);

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
        double difference = tempPose.getX() - robotPose.getX();
        if (difference > 0) {
            difference = difference * (1 - (double) 100 / 105);
            SmartDashboard.putNumber("dif", difference);

            SmartDashboard.putNumber("x before", tempPose.getX());
            Pose2d tempPose2 = tempPose
                    .plus(new Transform2d(new Translation2d(-difference, 0), new Rotation2d()));
            SmartDashboard.putNumber("x after", tempPose2.getX());
            tempPose = tempPose2;
        }
        driveOdometry.resetPosition(tempPose, getGyroscopeRotation());
        robotPose = tempPose;
    }

    public void resetHoldAngle() {
        holdAngleSetpoint = getGyroscopeRotation().getRadians();
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
}
