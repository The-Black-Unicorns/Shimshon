// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.blackunicornsswerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class DriveMotor {

private TalonFX motor;
private ModuleInfo moduleInfo;
private final static double falconToRPMCoefficient = 1 * 600.0 / 2048.0 * 24.0 / 34.0;


public DriveMotor(int CANid, ModuleInfo moduleInfo){
    motor = new TalonFX(CANid); 
    this.moduleInfo = moduleInfo;
}



public void setSpeed(double speedMs){
double speedWheelRpm = speedMs*60/(Math.PI*moduleInfo.getWheelDiameter());
double speedMotorRpm = speedWheelRpm/moduleInfo.getGearRatio();
double speedFalconUnits = speedMotorRpm/falconToRPMCoefficient;
motor.set(ControlMode.Velocity,speedFalconUnits);
}

public double getSpeed(){
    return ((motor.getSelectedSensorVelocity()*falconToRPMCoefficient)*moduleInfo.getGearRatio())*((Math.PI*moduleInfo.getWheelDiameter())/60);
}

}
