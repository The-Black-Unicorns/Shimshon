// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.blackunicornsswerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
    private Rotation2d steerOffset;

    private double sensorPositionCoefficient;
    private int resetIteration = 0;

    public SteerMotor (int motorCANid, int encoderCANid, ModuleInfo moduleInfo, double steerOffsetRadians) {
        this.moduleInfo = moduleInfo;
        encoder = new CANCoder(encoderCANid);
        motor = new TalonFX(motorCANid);
        steerOffset = new Rotation2d(steerOffsetRadians);

        sensorPositionCoefficient = 2.0 * Math.PI / 2048 * this.moduleInfo.getSteerReduction();

        configureEncoder(encoder, steerOffset);        
    }

    private void configureEncoder (CANCoder encoder, Rotation2d steerOffset){
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = steerOffset.getDegrees();
        config.sensorDirection = false; //Counter Clockwise

        CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed To Configure CANCoder");
    }

    public void setReferenceAngle(double referenceAngleRadians){


        //Setting Falcon encoder equal to absolute encoder 10 seconds after startup
        resetIteration++;
        if (resetIteration == 500){
            double absoluteAngleRadians = Math.toRadians(encoder.getAbsolutePosition());
            absoluteAngleRadians %= (2*Math.PI);
            if (absoluteAngleRadians <0){
                absoluteAngleRadians += 2*Math.PI;
            }
            motor.setSelectedSensorPosition(absoluteAngleRadians /sensorPositionCoefficient);
        }


        double currentAngleRadians = motor.getSelectedSensorPosition() * sensorPositionCoefficient;

        double currentAngleRadiansMod = currentAngleRadians % (2*Math.PI);
        if (currentAngleRadiansMod < 0){
            currentAngleRadians += 2*Math.PI;
        }

        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        motor.set(ControlMode.Position, adjustedReferenceAngleRadians /sensorPositionCoefficient);
    }

    public double getSteerAngle(){
        double currentAngleRadians = motor.getSelectedSensorPosition() * sensorPositionCoefficient;
        double currentAngleRadiansMod = currentAngleRadians % (2*Math.PI);
        if (currentAngleRadiansMod < 0){
            currentAngleRadians += 2*Math.PI;
        } 
        return currentAngleRadiansMod;
    }


}
