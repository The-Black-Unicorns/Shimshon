// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  public final static double falconToRPMCoefficient = 1 * 600.0 / 2048.0 * 24.0 / 34.0;
  TalonFX shooterMotor;
  double falconTargetRPM;


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotor = new TalonFX(Constants.SHOOTER_TALONFX_ID);
    falconTargetRPM = Constants.SHOOTER_FLYWHEEL_RPM_HIGH_GOAL;
  }

  public void spinShooter(){
    shooterMotor.set(ControlMode.Velocity, falconTargetRPM / falconToRPMCoefficient);
  }

  public void stopShooter(){
    shooterMotor.set(ControlMode.PercentOutput, 0);
  }

  public void reverseShooter(){
    shooterMotor.set(ControlMode.PercentOutput, Constants.REVERSE_FALCON_SPEED);
  }

  public boolean shooterReachedSpeed(){
    return Math.abs(falconTargetRPM - shooterMotor.getSelectedSensorVelocity() * falconToRPMCoefficient) <= Constants.SHOOTER_FLYWHEEL_RPM_TOLERANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
