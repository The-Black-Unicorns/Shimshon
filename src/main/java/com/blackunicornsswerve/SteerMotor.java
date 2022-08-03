// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.blackunicornsswerve;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.swervedrivespecialties.swervelib.ctre.CtreUtils;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class SteerMotor {

    private TalonFX motor;
    private CANCoder encoder;
    private ModuleInfo moduleInfo;
    public Rotation2d steerOffset;

    public SteerMotor (int motorCANid, int encoderCANid, ModuleInfo moduleInfo, double steerOffsetRadians) {
        this.moduleInfo = moduleInfo;
        encoder = new CANCoder(encoderCANid);
        motor = new TalonFX(motorCANid);
        steerOffset = new Rotation2d(steerOffsetRadians);

        configureEncoder(encoder, steerOffset);        
    }

    private void configureEncoder (CANCoder encoder, Rotation2d steerOffset){
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = steerOffset.getDegrees();
        config.sensorDirection = false; //Counter Clockwise

        CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed To Configure CANCoder");
    }



}
