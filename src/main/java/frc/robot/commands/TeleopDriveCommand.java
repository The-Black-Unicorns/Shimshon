package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class TeleopDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Joystick input;


    //Max acceleration m/s^2
    SlewRateLimiter Xfilter = new SlewRateLimiter(5);
    SlewRateLimiter Yfilter = new SlewRateLimiter(5);

    public TeleopDriveCommand(DrivetrainSubsystem drivetrainSubsystem, Joystick controller) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        input = controller;
        addRequirements(drivetrainSubsystem);

        changePID();
    }


    @Override
    public void execute() {
        ChassisSpeeds inputSpeed;

        double sensitivity = input.getRawAxis(4) / 2 + 0.5;
        inputSpeed = new ChassisSpeeds(
                Xfilter.calculate(deadband(input.getRawAxis(Constants.DRIVER_CONTROLLER_X_AXIS_ID), 0.05)
                        * sensitivity * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
                Yfilter.calculate(deadband(-input.getRawAxis(Constants.DRIVER_CONTROLLER_Y_AXIS_ID), 0.05)
                        * sensitivity * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
                deadband(-input.getRawAxis(Constants.DRIVER_CONTROLLER_Z_AXIS_ID), 0.05) * sensitivity
                        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        

        if (!input.getRawButton(1)) {
            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(inputSpeed.vxMetersPerSecond,
                            inputSpeed.vyMetersPerSecond, inputSpeed.omegaRadiansPerSecond,
                            GyroSubsystem.getInstance().getGyroscopeRotation()));
        } else {
            m_drivetrainSubsystem.drive(inputSpeed);
        } 
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(.0, 0.0, 0.0));
    }

    private double deadband(double value, double lowerLimit) {
        // return Math.abs(value) < lowerLimit ? 0 : value;
        if (Math.abs(value) < lowerLimit) {
            return 0;
        } else {

            if (value > 0) {
                return (value - lowerLimit) / (1 - lowerLimit);
            } else {
                return (value + lowerLimit) / (1 - lowerLimit);
            }
        }
    }

    public void changePID() {
        // double kP = input.getRawAxis(4) / 10 + Constants.SWERVE_ROTATION_KP;
        // double kD = input.getRawAxis(5) / 10 + Constants.SWERVE_ROTATION_KD;
        // 0.4, 0.05
        // m_drivetrainSubsystem.updatePID(kP, kD);
    }
}
