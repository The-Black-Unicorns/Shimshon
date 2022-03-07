// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PitTest;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestCommand extends SequentialCommandGroup {
  /** Creates a new TestCommand. */
  public TestCommand(DrivetrainSubsystem drivetrainSubsystem, BallSubsystem ballSubsystem, ClimberSubsystem climberSubsystem, PS4Controller testController) {
    
    
    addCommands(
      new WaitForButtonCommand(testController),
      new InstantCommand(ballSubsystem::openIntake, ballSubsystem),
      new WaitForButtonCommand(testController),
      new InstantCommand(ballSubsystem::closeIntake, ballSubsystem),
      new WaitForButtonCommand(testController)
    );
    
    addRequirements(drivetrainSubsystem, ballSubsystem, climberSubsystem);
  }
}
