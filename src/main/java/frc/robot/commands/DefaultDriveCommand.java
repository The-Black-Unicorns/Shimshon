package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Joystick input;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, Joystick controller) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        input = controller;

        addRequirements(drivetrainSubsystem);

        changePID();
    }

    @Override
    public void execute() {
        double sensitivity = input.getRawAxis(4) / 2 + 0.5;
        sensitivity = 0.25;
        SmartDashboard.putNumber("X", -input.getRawAxis(Constants.DRIVER_CONTROLLER_Y_AXIS_ID));
        SmartDashboard.putNumber("Y", input.getRawAxis(Constants.DRIVER_CONTROLLER_X_AXIS_ID));
        SmartDashboard.putNumber("Z", input.getRawAxis(Constants.DRIVER_CONTROLLER_Z_AXIS_ID));

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        if (!input.getRawButton(3)) {
            m_drivetrainSubsystem.drive(

                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            deadband(input.getRawAxis(Constants.DRIVER_CONTROLLER_Y_AXIS_ID), 0.05) * sensitivity
                                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                            deadband(-input.getRawAxis(Constants.DRIVER_CONTROLLER_X_AXIS_ID), 0.05) * sensitivity
                                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                            deadband(-input.getRawAxis(Constants.DRIVER_CONTROLLER_Z_AXIS_ID), 0.05) * sensitivity
                                    * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                            m_drivetrainSubsystem.getGyroscopeRotation()));
        } else {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(
                    deadband(input.getRawAxis(Constants.DRIVER_CONTROLLER_Y_AXIS_ID), 0.05) * sensitivity
                            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    deadband(-input.getRawAxis(Constants.DRIVER_CONTROLLER_X_AXIS_ID), 0.05) * sensitivity
                            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    deadband(-input.getRawAxis(Constants.DRIVER_CONTROLLER_Z_AXIS_ID), 0.05) * sensitivity
                            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
        }

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    private double deadband(double value, double lowerLimit) {
        return Math.abs(value) < lowerLimit ? 0 : value;
    }

    public void changePID() {
        // double kP = input.getRawAxis(4) / 10 + Constants.SWERVE_ROTATION_KP;
        // double kD = input.getRawAxis(5) / 10 + Constants.SWERVE_ROTATION_KD;
        // 0.4, 0.05
        // m_drivetrainSubsystem.updatePID(kP, kD);
    }
}
