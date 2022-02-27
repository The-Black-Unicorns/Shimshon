// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterKickerSubsystem extends SubsystemBase {

  TalonFX shooterFalcon;
  TalonSRX kickerWheel775;

  int falconUnitsTargetVelocity;
  boolean shooterReachedSpeed;
  boolean shooterSpinning;

  public ShooterKickerSubsystem() 
  {
    shooterFalcon = new TalonFX(Constants.SHOOTER_TALONFX_MOTOR);
    kickerWheel775 = new TalonSRX(Constants.KICKER_WHEEL_TALONSRX_MOTOR);

    DrivetrainSubsystem.updateFalconPID(Constants.SHOOTER_TALONFX_MOTOR, 0, 0, 0, 0.05, NeutralMode.Coast);
    setShooterSpeed(Constants.SHOOTER_FLYWHEEL_RPM_HIGH_GOAL);
  }

  @Override
  public void periodic() {
    if (Math.abs(shooterFalcon.getSelectedSensorVelocity() - falconUnitsTargetVelocity) < Constants.SHOOTER_FLYWHEEL_RPM_ERROR * 2048 / 600){
        shooterReachedSpeed = true;
    } else {
        shooterReachedSpeed = false;
    }
  }

  public void startFlywheel (){
    shooterFalcon.set(ControlMode.Velocity, falconUnitsTargetVelocity);
    shooterSpinning = true;
  }
 
  public void shooting (){
    if (shooterSpinning && shooterReachedSpeed){
      kickerWheel775.set(ControlMode.PercentOutput, Constants.KICKER_WHEEL_PERCENT);
    } else if (shooterSpinning){
      kickerWheel775.set(ControlMode.PercentOutput, 0);
      //Wait
    } else{
      startFlywheel();
    }
  }

  public void stopShooter(){
    shooterFalcon.set(ControlMode.PercentOutput, 0);
    kickerWheel775.set(ControlMode.PercentOutput, 0);
    shooterSpinning = false;
  }
  
  public void stopShooting(){
    kickerWheel775.set(ControlMode.PercentOutput, 0);   
  }

  public void setShooterSpeed(int rpmTarget){
    falconUnitsTargetVelocity = rpmTarget * 2048 / 600;    
  }

  
}
