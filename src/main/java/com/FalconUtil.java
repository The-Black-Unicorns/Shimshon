// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

/** Add your docs here. */
public class FalconUtil {
    public static void updatePID(int talonCanID, double kP, double kI, double kD, double kF,
            NeutralMode neutralMode, double maxCurrent) {
        TalonFXConfiguration talonConfiguration = new TalonFXConfiguration();
        talonConfiguration.slot0.kP = kP;
        talonConfiguration.slot0.kI = kI;
        talonConfiguration.slot0.kD = kD;
        talonConfiguration.slot0.kF = kF;
        TalonFX talon = new TalonFX(talonCanID);
        talon.configAllSettings(talonConfiguration);
        talon.setNeutralMode(neutralMode);
        talon.setStatusFramePeriod(1, 20);
        if (maxCurrent != 0){
            talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, maxCurrent, maxCurrent, 0.1));
        }
    }
}
