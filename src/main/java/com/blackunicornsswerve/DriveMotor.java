// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.blackunicornsswerve;


/** Add your docs here. */
public class DriveMotor {

private int id;
private motorLocation mLocation;


public enum motorLocation
{   
    BottomRight,
    BottomLeft,
    UpperRight,
    UpperLeft;

    private motorLocation mLocation;

    public motorLocation setMotorLocation(motorLocation mLocation){
        this.mLocation = mLocation;
        return mLocation;
    }
    
    public motorLocation getLocation(){
        return mLocation;
    }
}
public DriveMotor(int id,motorLocation mLocation){

    this.id =id;
    this.mLocation = mLocation.setMotorLocation(mLocation); 
}

public double getDriveVelocityRpmToMs(double rpm, ModuleInfo module){
    return rpm*Math.PI*module.wheelDiameter();
    }
public void setSpeedRPM(int RPM){}
public void setSpeedMS(double ms){}

}
