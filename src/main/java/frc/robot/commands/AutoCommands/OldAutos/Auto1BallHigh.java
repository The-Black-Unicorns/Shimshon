// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.OldAutos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TrajectoryFollowingCommand;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1BallHigh extends ParallelCommandGroup {
  /** Creates a new Auto1Ball. */
  public Auto1BallHigh(BallSubsystem ballSubsystem, DrivetrainSubsystem drivetrainSubsystem, String bluePathName, String redPathName, double speedMultiplier) {
    addCommands(new Auto1BallHighSystems(ballSubsystem));
    if (DriverStation.getAlliance() == Alliance.Blue){
      addCommands(sequence(new WaitCommand(4), new TrajectoryFollowingCommand(drivetrainSubsystem, bluePathName, speedMultiplier)));
    } else {
      addCommands(sequence(new WaitCommand(4), new TrajectoryFollowingCommand(drivetrainSubsystem, redPathName, speedMultiplier)));
    }
    
    addRequirements(ballSubsystem, drivetrainSubsystem);
  }
}
