package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Joystick input;
    private final PS4Controller alternateDriveController;

    //Max acceleration m/s^2
    SlewRateLimiter Xfilter = new SlewRateLimiter(1000);
    SlewRateLimiter Yfilter = new SlewRateLimiter(1000);

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, Joystick controller, PS4Controller alternateController) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        input = controller;
        alternateDriveController = alternateController;
        addRequirements(drivetrainSubsystem);

        changePID();
    }

    @Override
    public void execute() {
        ChassisSpeeds inputSpeed;
        if(!Constants.DRIVER_MODE){
            double sensitivity = input.getRawAxis(4) / 2 + 0.5;
            // sensitivity = 0.25;
             inputSpeed = new ChassisSpeeds(
                Xfilter.calculate(deadband(input.getRawAxis(Constants.DRIVER_CONTROLLER_X_AXIS_ID), 0.05) * sensitivity * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
                Yfilter.calculate(deadband(-input.getRawAxis(Constants.DRIVER_CONTROLLER_Y_AXIS_ID), 0.05) * sensitivity * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
                deadband(-input.getRawAxis(Constants.DRIVER_CONTROLLER_Z_AXIS_ID), 0.05) * sensitivity * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    
        }
        else{
            double angle = Math.atan2(deadband(alternateDriveController.getLeftY(), 0.05),deadband(alternateDriveController.getLeftX(), 0.05));
            double speed = ((alternateDriveController.getR2Axis()+1)/2)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
            System.out.println(speed);
            double rotation = deadband(alternateDriveController.getRightX(),0.05)*DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
            double xSpeed = Math.cos(angle)*speed;
            double ySpeed = Math.sin(angle)*speed;
            inputSpeed = new ChassisSpeeds(ySpeed,xSpeed,rotation);
        }

        if (!input.getRawButton(5)) {
            if (!input.getRawButton(1)) {
                m_drivetrainSubsystem.drive(
                        ChassisSpeeds.fromFieldRelativeSpeeds(inputSpeed.vxMetersPerSecond, inputSpeed.vyMetersPerSecond, inputSpeed.omegaRadiansPerSecond, GyroSubsystem.getInstance().getGyroscopeRotation()));
            } else {
                m_drivetrainSubsystem.drive(inputSpeed);
                        
            }
        } else {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
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
