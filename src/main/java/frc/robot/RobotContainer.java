// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.TrajectoryFollowingCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

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
  //Controllers
  private final Joystick controller = new Joystick(0);
  //Commands
  private final DefaultDriveCommand driveCommand = new DefaultDriveCommand(drivetrainSubsystem, controller);
  private final TrajectoryFollowingCommand trajectoryCommand = new TrajectoryFollowingCommand(drivetrainSubsystem, controller);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    drivetrainSubsystem.setDefaultCommand(driveCommand);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new JoystickButton(controller, 1)
        // No requirements because we don't need to interrupt anything
        .whenPressed(() -> drivetrainSubsystem.zeroGyroscope());
    
    new JoystickButton(controller, 2)
        .whenPressed(() -> CommandScheduler.getInstance().schedule(trajectoryCommand));
    
    new JoystickButton(controller, 4)
        .whenPressed(() -> drivetrainSubsystem.matchEncoders());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
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
    drivetrainSubsystem.resetHoldAngle();
    

    
  }

  public void onDisable() {

  }
}
