// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSubsystem;

public class BallTeleopCommand extends CommandBase {

  BallSubsystem ballSubsystem;
  XboxController controller;

  public BallTeleopCommand(BallSubsystem subsystem, XboxController secondDriverController) {

    ballSubsystem = subsystem;
    controller = secondDriverController;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (controller.getLeftBumperPressed()){
      ballSubsystem.prepareForShootingInit();
    } else if (controller.getLeftBumper()){
      ballSubsystem.prepareForShootingPereodic();
    }


    if (controller.getRightTriggerAxis() >= 0.2){
      ballSubsystem.shoot();
    } else if (!controller.getLeftBumper()){
      ballSubsystem.stopShooter();
    }
  }

  public void closeIntake(){
    ballSubsystem.closeIntake();
  }

  public void openIntake(){
    ballSubsystem.openIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
