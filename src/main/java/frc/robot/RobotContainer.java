// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BallTeleopCommand;
import frc.robot.commands.ClimberTeleopCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.TrajectoryFollowingCommand;
import frc.robot.commands.AutoCommands.Auto1BallLeft;
import frc.robot.commands.AutoCommands.Auto2Ball;
import frc.robot.commands.AutoCommands.Auto3Ball;
import frc.robot.commands.PitTest.TestCommand;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, comman
 * ds, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final BallSubsystem ballSubsystem = new BallSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    // Controllers
    private final Joystick controller = new Joystick(0);

    private final PS4Controller secondDriverController = new PS4Controller(1);
    private final PS4Controller alternateDriveController = new PS4Controller(2);

    // Commands
    private final DefaultDriveCommand driveCommand = new DefaultDriveCommand(drivetrainSubsystem, controller,
            alternateDriveController);
    private final BallTeleopCommand ballTeleopCommand = new BallTeleopCommand(ballSubsystem, secondDriverController,
            controller);
    private final ClimberTeleopCommand climberCommand = new ClimberTeleopCommand(climberSubsystem, controller,
            secondDriverController);

    // Autonomus Commands
    private final Auto1BallLeft auto1BallLeft = new Auto1BallLeft(ballSubsystem, drivetrainSubsystem,
            "1 Ball Auto Red Left", "1 Ball Auto Red Left", 0.2);
    private final Auto2Ball auto2Ball = new Auto2Ball(ballSubsystem, drivetrainSubsystem, "2 Ball Auto Red",
            "2 Ball Auto Red", 0.2);
    private final Auto3Ball auto3Ball = new Auto3Ball(ballSubsystem, drivetrainSubsystem, "3 Ball Auto Red",
            "3 Ball Auto Red", 0.3);

    // private final Auto1BallHigh auto1BallHigh = new Auto1BallHigh(ballSubsystem,
    // drivetrainSubsystem, "1 Ball Auto High", "1 Ball Auto High", 0.2);
    // private final Auto2BallHigh auto2BallHigh = new Auto2BallHigh(ballSubsystem,
    // drivetrainSubsystem, "2 Ball Auto High", "2 Ball Auto High", 0.2);
    // private final Auto3BallHigh auto3BallHigh = new Auto3BallHigh(ballSubsystem,
    // drivetrainSubsystem, "3 Ball Auto High", "3 Ball Auto High", 0.3);
    // private final Auto4BallHigh auto4Ball = new Auto4BallHigh(ballSubsystem,
    // drivetrainSubsystem, "4 Ball Auto Red Part 1", "4 Ball Auto Red Part 2", "4
    // Ball Auto Red Part 1", "4 Ball Auto Red Part 2", 0.6);

    private final InstantCommand noAuto = new InstantCommand();

    private final TrajectoryFollowingCommand taxiAuto = new TrajectoryFollowingCommand(drivetrainSubsystem, "Taxi left",
            0.2);

    // Test commands
    private final TestCommand test = new TestCommand(drivetrainSubsystem, ballSubsystem, climberSubsystem,
            secondDriverController);

    @Log
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, false);

        drivetrainSubsystem.setDefaultCommand(driveCommand);
        ballSubsystem.setDefaultCommand(ballTeleopCommand);
        climberSubsystem.setDefaultCommand(climberCommand);

        // Enable or disable compressor
        Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
        phCompressor.enableDigital();
        // phCompressor.disable();
        phCompressor.close();

        // Autonomus Chooser
        autoChooser.addOption("1 Ball Left", auto1BallLeft);
        autoChooser.addOption("2 Ball", auto2Ball);
        autoChooser.addOption("3 Ball", auto3Ball);
        autoChooser.addOption("No Auto", noAuto);
        autoChooser.addOption("Taxi", taxiAuto);

        // SmartDashboard.putData(autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * 
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Back button zeros the gyroscope
        new JoystickButton(controller, 3)
                // No requirements because we don't need to interrupt anything
                .whenPressed(() -> drivetrainSubsystem.zeroPosition());

        new JoystickButton(alternateDriveController, PS4Controller.Button.kCross.value)
                .whenPressed(() -> drivetrainSubsystem.zeroPosition());

        new JoystickButton(controller, 4)
                .whenPressed(() -> drivetrainSubsystem.matchEncoders());

        new JoystickButton(secondDriverController, PS4Controller.Button.kCross.value)
                .whenPressed(() -> ballTeleopCommand.openIntake());

        new JoystickButton(secondDriverController, PS4Controller.Button.kCircle.value)
                .whenPressed(() -> ballTeleopCommand.closeIntake());

        new JoystickButton(secondDriverController, PS4Controller.Button.kL1.value)
                .whenPressed(() -> climberSubsystem.toggleOutsideSolenoid());

        new JoystickButton(secondDriverController, PS4Controller.Button.kR1.value)
                .whenPressed(() -> climberSubsystem.toggleInsideSolenoid());

        new JoystickButton(secondDriverController, PS4Controller.Button.kTouchpad.value)
                .whenPressed(() -> ballSubsystem.startSpittingShooter())
                .whenReleased(() -> ballSubsystem.stopSpittingShooter());

        new JoystickButton(secondDriverController, PS4Controller.Button.kPS.value)
                .whenPressed(() -> climberSubsystem.startOutsideOpen());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }

    /*
     * private static double deadband(double value, double deadband) {
     * if (Math.abs(value) > deadband) {
     * if (value > 0.0) {
     * return (value - deadband) / (1.0 - deadband);
     * } else {
     * return (value + deadband) / (1.0 - deadband);
     * }
     * } else {
     * return 0.0;
     * }
     * }
     * 
     * private static double modifyAxis(double value) {
     * // Deadband
     * value = deadband(value, 0.05);
     * 
     * // Square the axis
     * value = Math.copySign(value * value, value);
     * 
     * return value;
     * }
     */

    public void onEnable() {
        drivetrainSubsystem.onEnable();
        ballSubsystem.onEnable();
        climberSubsystem.onEnable();
    }

    public void onDisable() {
        drivetrainSubsystem.onDisable();
        ballSubsystem.onDisable();
    }

    public void onTestInit() {
        test.schedule();
    }

    public void robotPeriodic() {
        Logger.updateEntries();
        GyroSubsystem.getInstance().updateGyroAngle();
    }

    @Config(tabName = Constants.MAIN_DASHBOARD_TAB_NAME, name = "Alternate Drive", defaultValueBoolean = false)
    public void setAlternateDrive(boolean value) {
        driveCommand.setAlternateDrive(value);
    }
    @Config(tabName = Constants.MAIN_DASHBOARD_TAB_NAME, name = "Field Oriented", defaultValueBoolean = true)
    public void setFieldOriented(boolean value) {
        driveCommand.setFieldOriented(value);
    }
    @Config(tabName = Constants.MAIN_DASHBOARD_TAB_NAME, name = "Hold Angle", defaultValueBoolean = true)
    public void setHoldAngleMode(boolean value) {
        drivetrainSubsystem.setHoldAngleMode(value);
    }
    @Config (tabName = Constants.MAIN_DASHBOARD_TAB_NAME, name = "Extra stop", defaultValueBoolean = true)
    public void setExtraBrake(boolean value) {
        setExtraBrake(value);;
    }
    @Log (tabName = Constants.MAIN_DASHBOARD_TAB_NAME, name = "Shooter RPM")
    public double getShooterRPM(){
        return ballSubsystem.getShooterRPM();
    }
}
