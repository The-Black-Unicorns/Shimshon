// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSubsystem;

public class BallTeleopCommand extends CommandBase {

  BallSubsystem ballSubsystem;
  PS4Controller controller;

  double triggerThreashold = 0.3;

  private double previousRightTrigger;
  private double previousLeftTrigger;

  public BallTeleopCommand(BallSubsystem subsystem, PS4Controller secondDriverController) {

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
    double rightTrigger = controller.getR2Axis();
    double leftTrigger = controller.getL2Axis();


    //Triggers detection
    if (rightTrigger >= triggerThreashold && previousRightTrigger < triggerThreashold)
    {
      rightTriggerPressed();
    }
    if (rightTrigger >= triggerThreashold && previousRightTrigger >= triggerThreashold)
    {
      rightTrigger();
    }
    if (rightTrigger < triggerThreashold && previousRightTrigger >= triggerThreashold){
      rightTriggerReleased();
    }
    if (leftTrigger >= triggerThreashold && previousLeftTrigger < triggerThreashold)
    {
      leftTriggerPressed();
    }
    if (leftTrigger >= triggerThreashold && previousLeftTrigger >= triggerThreashold)
    {
      leftTrigger();
    }
    if (leftTrigger < triggerThreashold && previousLeftTrigger >= triggerThreashold){
      leftTriggerReleased();
    }
    previousRightTrigger = rightTrigger;
    previousLeftTrigger = leftTrigger;



  }

  void rightTriggerPressed ()
  {
    
  }

  void rightTrigger ()
  {
    ballSubsystem.shoot();
  }

  void rightTriggerReleased(){
    if (controller.getL2Axis() < triggerThreashold){
      ballSubsystem.stopShooter();
    }
  }

  void leftTriggerPressed ()
  {
    ballSubsystem.prepareForShootingInit();
  }

  void leftTrigger ()
  {
  }

  void leftTriggerReleased(){
    if (controller.getR2Axis() < triggerThreashold){
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
