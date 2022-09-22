package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class DefaultDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Joystick input;
    private final PS4Controller alternateDriveController;

    private boolean isFieldOriented;
    private boolean isAlternateDriveControl;

    //Max acceleration m/s^2
    SlewRateLimiter Xfilter = new SlewRateLimiter(6);
    SlewRateLimiter Yfilter = new SlewRateLimiter(6);
    SlewRateLimiter speedLimiter = new SlewRateLimiter(1000);

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, Joystick controller, PS4Controller alternateController) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        input = controller;
        alternateDriveController = alternateController;
        addRequirements(drivetrainSubsystem);

        changePID();
    }
    // @Config (tabName = Constants.MAIN_DASHBOARD_TAB_NAME, name = "Field Oriented", defaultValueBoolean = true)
    public void setFieldOriented(boolean value){
        isFieldOriented = value;
    }

    // @Config (tabName = Constants.MAIN_DASHBOARD_TAB_NAME, name = "Alternate Drive", defaultValueBoolean = false)
    public void setAlternateDrive(boolean value){
        isAlternateDriveControl = value;
    }

    
    @Override
    public void execute() {
        ChassisSpeeds inputSpeed;
        if(!isAlternateDriveControl){
            double sensitivity = input.getRawAxis(4) / 2 + 0.5;
            // sensitivity = 0.25;
             inputSpeed = new ChassisSpeeds(
                Xfilter.calculate(deadband(input.getRawAxis(Constants.DRIVER_CONTROLLER_X_AXIS_ID), 0.05) * sensitivity * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
                Yfilter.calculate(deadband(-input.getRawAxis(Constants.DRIVER_CONTROLLER_Y_AXIS_ID), 0.05) * sensitivity * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
                deadband(-input.getRawAxis(Constants.DRIVER_CONTROLLER_Z_AXIS_ID), 0.05) * sensitivity * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
            // System.out.println(inputSpeed.toString());
    
        }
        else{
            double xInput = deadband(alternateDriveController.getRightX(), 0.05);
            double yInput = deadband(alternateDriveController.getRightY(), 0.05);
            double angle = Math.atan2(yInput, xInput);
            double speed = speedLimiter.calculate(Math.pow(((alternateDriveController.getR2Axis()+1)/2)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 1))  ;
            if (xInput == 0 && yInput == 0){
                speed = 0;
            }
            System.out.println("Test " + speed);
            double rotation = deadband(-alternateDriveController.getLeftX(),0.001)*DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
            double xSpeed = Xfilter.calculate(-Math.cos(angle)*speed);
            double ySpeed = Yfilter.calculate(-Math.sin(angle)*speed);
            inputSpeed = new ChassisSpeeds(ySpeed,xSpeed,rotation);
        }

        if (!input.getRawButton(5)) {
            if (isFieldOriented) {
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
