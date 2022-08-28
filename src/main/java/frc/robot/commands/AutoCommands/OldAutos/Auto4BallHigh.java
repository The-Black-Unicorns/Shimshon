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
public class Auto4BallHigh extends ParallelCommandGroup {
  /** Creates a new Auto1Ball. */
  public Auto4BallHigh(BallSubsystem ballSubsystem, DrivetrainSubsystem drivetrainSubsystem, String bluePath1Name, String bluePath2Name, String redPath1Name, String redPath2Name, double speedMultiplier) {
    addCommands(new Auto4BallHighSystems(ballSubsystem));
    if (DriverStation.getAlliance() == Alliance.Blue){
      addCommands(sequence(new WaitCommand(0.5), new TrajectoryFollowingCommand(drivetrainSubsystem, bluePath1Name, 0.3), 
                            new WaitCommand(1), 
                            new TrajectoryFollowingCommand(drivetrainSubsystem, bluePath2Name, speedMultiplier)));
    } else {
      addCommands(sequence(new WaitCommand(0.5), new TrajectoryFollowingCommand(drivetrainSubsystem, redPath1Name, 0.3), 
                            new WaitCommand(1), 
                            new TrajectoryFollowingCommand(drivetrainSubsystem, redPath2Name, speedMultiplier)));
    }
    
    addRequirements(ballSubsystem, drivetrainSubsystem);
  }
}
