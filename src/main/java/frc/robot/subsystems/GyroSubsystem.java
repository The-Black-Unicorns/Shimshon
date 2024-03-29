// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class GyroSubsystem {

    private static GyroSubsystem gyroSubsystemInstance;

    private final PigeonIMU pigeon = new PigeonIMU(Constants.PIGEON_ID);

    private Rotation2d gyroAngle;
    private Rotation2d gyroOffset = new Rotation2d();

    private GyroSubsystem() {
        // Pigeon status frames
        pigeon.setStatusFramePeriod(6, 10);
        pigeon.setStatusFramePeriod(11, 10);
        pigeon.setStatusFramePeriod(4, 10);
    }

    public static GyroSubsystem getInstance() {

        if (gyroSubsystemInstance == null)
            gyroSubsystemInstance = new GyroSubsystem();
        
        return gyroSubsystemInstance;
    }

    private Rotation2d getGyroRotationRaw() {
        return Rotation2d.fromDegrees(pigeon.getFusedHeading() * 1.00278552);
    }

    public void updateGyroAngle() {
        gyroAngle = getGyroRotationRaw().minus(gyroOffset);
    }

    public Rotation2d getGyroscopeRotation() {
        return gyroAngle;
    }

    public void zeroGyro(Rotation2d newRotation) {
        gyroOffset = getGyroRotationRaw().minus(newRotation);
    }

    public void zeroGyro() {
        zeroGyro(new Rotation2d());
    }

}
