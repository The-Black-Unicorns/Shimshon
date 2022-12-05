// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  TalonSRX intakeMotor;
  DoubleSolenoid intakePiston;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonSRX(Constants.INTAKE_TALONSRX_ID);
    intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);

  }

  public void set(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void openPiston(){
    intakePiston.set(Value.kForward);
  }

  public void closePiston(){
    intakePiston.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
