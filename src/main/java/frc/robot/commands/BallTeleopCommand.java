// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallSubsystem;

public class BallTeleopCommand extends CommandBase {

  BallSubsystem ballSubsystem;
  PS4Controller controller;
  Joystick mainController;


  double triggerThreashold = 0.3;

  private double previousRightTrigger;
  private double previousLeftTrigger;
  private int previousPov;

  public BallTeleopCommand(BallSubsystem subsystem, PS4Controller secondDriverController, Joystick mainController) {

    ballSubsystem = subsystem;
    controller = secondDriverController;
    this.mainController = mainController;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    ballSubsystem.setShooterSpeed(Constants.SHOOTER_FLYWHEEL_RPM_HIGH_GOAL + (int)(mainController.getRawAxis(5) * 100));

    //Triggers detection
    double rightTrigger = controller.getR2Axis();
    if (rightTrigger >= triggerThreashold && previousRightTrigger < triggerThreashold){
      rightTriggerPressed();
    }
    if (rightTrigger >= triggerThreashold && previousRightTrigger >= triggerThreashold){
      rightTrigger();
    }
    if (rightTrigger < triggerThreashold && previousRightTrigger >= triggerThreashold){
      rightTriggerReleased();
    }
    previousRightTrigger = rightTrigger;

    double leftTrigger = controller.getL2Axis();
    if (leftTrigger >= triggerThreashold && previousLeftTrigger < triggerThreashold){
      leftTriggerPressed();
    }
    if (leftTrigger >= triggerThreashold && previousLeftTrigger >= triggerThreashold){
      leftTrigger();
    }
    if (leftTrigger < triggerThreashold && previousLeftTrigger >= triggerThreashold){
      leftTriggerReleased();
    }
    previousLeftTrigger = leftTrigger;

    int pov = controller.getPOV();
    if ((pov == 0 || pov == 45 || pov == 315) && (previousPov != 0 && previousPov != 45 && previousPov != 315)){
      upPressed();
    } else if ((pov != 0 && pov != 45 && pov != 315) && (previousPov == 0 || previousPov == 45 || previousPov == 315)){
      upReleased();
    } else if ((pov == 180 || pov == 135 || pov == 225) && (previousPov != 180 && previousPov != 135 && previousPov != 225)){
      ballSubsystem.startReversingIntake();
    } else if ((pov != 180 && pov != 135 && pov != 225) && (previousPov == 180 || previousPov == 135 || previousPov == 225)){
      ballSubsystem.stopReversingIntake();
    }

    // if (pov ==  0 && previousPov != 0){
    //   upPressed();
    // } else if (pov != 0 & previousPov == 0){
    //   upReleased();
    // } else if (pov == 180 & previousPov != 180){
    //   ballSubsystem.startReversingIntake();
    // } else if (pov != 180 && previousPov == 180){
    //   ballSubsystem.stopReversingIntake();
    // }

    previousPov = pov;
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

  void upPressed (){
    ballSubsystem.setShooterSpeed(Constants.SHOOTER_FLYWHEEL_RPM_HIGH_GOAL);
  }

  void upReleased (){
    ballSubsystem.setShooterSpeed(Constants.SHOOTER_FLYWHEEL_RPM_LOW_GOAL);
  }

  public void closeIntake(){
    ballSubsystem.closeIntake(false);
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
