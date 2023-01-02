package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class AlternateDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final PS4Controller alternateDriveController;


    //Max acceleration m/s^2
    SlewRateLimiter Xfilter = new SlewRateLimiter(5);
    SlewRateLimiter Yfilter = new SlewRateLimiter(5);
    SlewRateLimiter speedLimiter = new SlewRateLimiter(1000);

    public AlternateDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            PS4Controller alternateController) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        alternateDriveController = alternateController;
        addRequirements(drivetrainSubsystem);

        changePID();
    }


    @Override
    public void execute() {
        ChassisSpeeds inputSpeed;
        double xInput = deadband(alternateDriveController.getRightX(), 0.1);
        double yInput = deadband(alternateDriveController.getRightY(), 0.1);
        double angle = Math.atan2(yInput, xInput);
        double speed = speedLimiter.calculate(Math.pow(((alternateDriveController.getR2Axis() + 1) / 2)
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 1) * 0.3);
        if (xInput == 0 && yInput == 0) {
            speed = 0;
        }
        double rotation = deadband(-alternateDriveController.getLeftX(), 0.1) * 0.3
                * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        double xSpeed = Xfilter.calculate(-Math.cos(angle) * speed);
        double ySpeed = Yfilter.calculate(-Math.sin(angle) * speed);
        inputSpeed = new ChassisSpeeds(ySpeed, xSpeed, rotation);

        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(inputSpeed.vxMetersPerSecond,
                        inputSpeed.vyMetersPerSecond, inputSpeed.omegaRadiansPerSecond,
                        GyroSubsystem.getInstance().getGyroscopeRotation()));
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
