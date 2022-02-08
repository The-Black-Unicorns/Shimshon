// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ShooterSubsystem extends SubsystemBase {

  TalonFX shooterFalcon;
  TalonSRX kickerWheel775;

  int velocityFalconUnitsHighGoal;
  boolean shooterReachedSpeed;

  public ShooterSubsystem() 
  {
    shooterFalcon = new TalonFX(Constants.SHOOTER_TALONFX_MOTOR);
    kickerWheel775 = new TalonSRX(Constants.KICKER_WHEEL_TALONSRX_MOTOR);

    DrivetrainSubsystem.updateFalconPID(Constants.SHOOTER_TALONFX_MOTOR, 0, 0, 0, 0.05, NeutralMode.Coast);
    velocityFalconUnitsHighGoal = Constants.SHOOTER_FLYWHEEL_RPM_HIGH_GOAL * 2048 / 600;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startFlywheel (){
    shooterFalcon.set(ControlMode.Velocity, velocityFalconUnitsHighGoal);
  }

  public void startShooting (){

  }

}
