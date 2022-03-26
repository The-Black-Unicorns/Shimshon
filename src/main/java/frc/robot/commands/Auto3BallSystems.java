// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSubsystem;

public class Auto3BallSystems extends CommandBase {

  BallSubsystem ballSubsystem;

  int counter = 0;

  public Auto3BallSystems(BallSubsystem subsystem) {

    ballSubsystem = subsystem;
    addRequirements(subsystem);

  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (counter < 100){
      ballSubsystem.shoot();
    } else if (counter == 100){
      ballSubsystem.stopShooter();
      ballSubsystem.openIntake();
    // } else if (counter == 280){
    //   ballSubsystem.startReversingIntake();
    // } else if (counter == 330){
    //   ballSubsystem.stopReversingIntake();
    // } else if (counter == 450){
    //   ballSubsystem.startReversingIntake();
    // } else if (counter == 475){
    //   ballSubsystem.stopReversingIntake();
    } else if (counter > 550){
      ballSubsystem.shoot();
    }
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter == 750;
  }
}
