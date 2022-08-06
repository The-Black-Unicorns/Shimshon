// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.blackunicornsswerve;


/** Add your docs here. */
public class ModuleInfo {
    private final double wheelDiameter;
    private final double gearRatio;
    private final double maxSpeed;
    private final double steerReduction;

    public ModuleInfo (double wheelDiameter, double gearRatio, double maxSpeed, double steerReduction){
        this.wheelDiameter = wheelDiameter;
        this.gearRatio = gearRatio;
        this.maxSpeed = maxSpeed;
        this.steerReduction = steerReduction;
    }

    public double getGearRatio(){
        return gearRatio;
    }

    public double getWheelDiameter(){
        return wheelDiameter;
    }

    public double getMaxSpeed(){
        return maxSpeed;
    }

    public double getSteerReduction (){
        return steerReduction;
    }


    public ModuleInfo Mk4_L1 = new ModuleInfo(0.1016, 1/8.14,4.118, 1/12.8);
    public ModuleInfo Mk4_L2 = new ModuleInfo(0.1016, 1/6.75,4.9682, 1/12.8);
    public ModuleInfo Mk4_L3 = new ModuleInfo(0.1016, 1/6.12,5.486, 1/12.8);
    public ModuleInfo Mk4_L4 = new ModuleInfo(0.1016, 1/5.14,6.522, 1/12.8);
}
