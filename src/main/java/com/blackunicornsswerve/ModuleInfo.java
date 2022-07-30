// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.blackunicornsswerve;


/** Add your docs here. */
public class ModuleInfo {
    private final double wheelDiameter;
    private final double gearRatio;

    public ModuleInfo (double wheelDiameter, double gearRatio){
        this.wheelDiameter = wheelDiameter;
        this.gearRatio = gearRatio;
    }

    public double getGearRatio(){
        return gearRatio;
    }

    public double wheelDiameter(){
        return wheelDiameter;
    }

    public ModuleInfo Mk4_L1 = new ModuleInfo(0.1016, 1/8.14);
    public ModuleInfo Mk4_L2 = new ModuleInfo(0.1016, 1/6.75);
    public ModuleInfo Mk4_L3 = new ModuleInfo(0.1016, 1/6.12);
    public ModuleInfo Mk4_L4 = new ModuleInfo(0.1016, 1/5.14);
}
