// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallSubsystem extends SubsystemBase {

  TalonFX shooterFalcon;
  TalonSRX conveyor775;
  TalonSRX intake775;
  DoubleSolenoid intakeSolenoid;
  PowerDistribution pdp;

  int falconUnitsTargetVelocity;
  boolean shooterReachedSpeed;
  int conveyorReverseTimer;
  boolean ballAtBarrel = false;
  boolean intakeOpen = false;
  int framesSinceIntakeOpen = 0;
  int framesSinceIntakeClosed = 0;
  boolean shooterWarming = false;

  public BallSubsystem() 
  {
    shooterFalcon = new TalonFX(Constants.SHOOTER_TALONFX_MOTOR);
    conveyor775 = new TalonSRX(Constants.CONVEYOR_TALONSRX_MOTOR);
    intake775 = new TalonSRX(Constants.INTAKE_TALONSRX_MOTOR);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
    pdp = new PowerDistribution(0, ModuleType.kCTRE);

    // DrivetrainSubsystem.updateFalconPID(Constants.SHOOTER_TALONFX_MOTOR, 0, 0, 0, 0.05, NeutralMode.Coast);
    setShooterSpeed(Constants.SHOOTER_FLYWHEEL_RPM);
    closeIntake();
  }

  public void openIntake (){
    intakeOpen = true;
    ballAtBarrel = true;
    conveyor775.set(ControlMode.PercentOutput, Constants.CONVEYOR_SPEED_PERCENT);
    intakeSolenoid.set(Value.kForward);
    framesSinceIntakeOpen = 0;
    framesSinceIntakeClosed = Integer.MIN_VALUE;
  }

  public void closeIntake(){
    intake775.set(ControlMode.PercentOutput, 0);
    conveyor775.set(ControlMode.PercentOutput, 0);
    intakeSolenoid.set(Value.kReverse);
    framesSinceIntakeOpen = Integer.MIN_VALUE;
    framesSinceIntakeClosed = 0;
    intakeOpen = false;
  }

  public void prepareForShootingInit(){
    if (intakeOpen)
    closeIntake();

    shooterWarming = true;
    conveyor775.set(ControlMode.PercentOutput, Constants.CONVEYOR_SPEED_PERCENT_REVERSE);
    conveyorReverseTimer = Constants.CONVEYER_REVERSE_DURATION_FRAMES;
    shooterFalcon.set(ControlMode.PercentOutput, -0.4);
  }

  public void setShooterSpeed(int rpmTarget){
    falconUnitsTargetVelocity = rpmTarget * 2048 / 600;    
  }

  public void shoot()
  {
    if (shooterReachedSpeed && !ballAtBarrel)
    {
      conveyor775.set(ControlMode.PercentOutput, Constants.CONVEYOR_SPEED_PERCENT);
    }  else if (!shooterWarming){
      prepareForShootingInit();
    } 
  }

  public void stopShooter (){
    if (!intakeOpen)
    conveyor775.set(ControlMode.PercentOutput, 0);

    shooterWarming = false;
    shooterFalcon.set(ControlMode.PercentOutput, 0);
    conveyorReverseTimer = -1;
  }

  @Override
  public void periodic() {

    //Checking if reached speed
    if (Math.abs(shooterFalcon.getSelectedSensorVelocity() - falconUnitsTargetVelocity) < Constants.SHOOTER_FLYWHEEL_RPM_ERROR * 2048 / 600){
      shooterReachedSpeed = true;
    } else {
      shooterReachedSpeed = false;
    }

    //RPM on dashboard
    SmartDashboard.putNumber("RPM", shooterFalcon.getSelectedSensorVelocity() * 600 /2048);

    //Waiting to spin the intake
    if (framesSinceIntakeOpen == 20 ){
      intake775.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED_PERCENT);
    }

    //Soften intake open and close

    // if (framesSinceIntakeOpen == 10){
    //   intakeSolenoid.set(Value.kReverse);
    // } else if (framesSinceIntakeOpen ==17){
    //   intakeSolenoid.set(Value.kForward);
    // } else if (framesSinceIntakeOpen == 18){
    //   intakeSolenoid.set(Value.kOff);
    // }
    if (framesSinceIntakeClosed == 15){
      intakeSolenoid.set(Value.kForward);

    } else if (framesSinceIntakeClosed ==20){
      intakeSolenoid.set(Value.kReverse);
    } else if (framesSinceIntakeClosed == 21 ){
      intakeSolenoid.set(Value.kReverse);
    }

    //Close intake when ball stuck
    if (framesSinceIntakeOpen >= 50 && pdp.getCurrent(7) > 22 && conveyorReverseTimer < 0 && intakeOpen){
      closeIntake();
    }

    //Stop warming up when finished warming up
    if (conveyorReverseTimer == 0){
      shooterFalcon.set(ControlMode.Velocity, falconUnitsTargetVelocity);
      conveyor775.set(ControlMode.PercentOutput, 0);
      ballAtBarrel = false;
    }

    framesSinceIntakeOpen++;
    framesSinceIntakeClosed++;
    conveyorReverseTimer--;
    
  }

  
}
