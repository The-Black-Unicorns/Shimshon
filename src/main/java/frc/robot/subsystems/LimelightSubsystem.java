// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class LimelightSubsystem {

    private static LimelightSubsystem limelightSubsystemInstance;

    private NetworkTableEntry tvEntry; // Valid targets
    private NetworkTableEntry txEntry; // X angle
    private NetworkTableEntry tyEntry; // Y angle
    private NetworkTableEntry tlEntry; // Latency

    private LimelightSubsystem() {
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        tvEntry = limelightTable.getEntry("tv");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        tlEntry = limelightTable.getEntry("tl");
    }

    public static LimelightSubsystem getInstance() {
        if (limelightSubsystemInstance == null)
            limelightSubsystemInstance = new LimelightSubsystem();

        return limelightSubsystemInstance;
    }

    public boolean hasTarget() {
        if (tvEntry.getDouble(0) == 0) {
            return false;
        } else {
            return true;
        }
    }

    public double getXAngle() {
        return txEntry.getDouble(0);
    }

    public double getYAngle() {
        return tyEntry.getDouble(0);
    }

    public double getCameraDelayMs() {
        return tlEntry.getDouble(0) + 11;
    }

    public double getDistanceMetersVision() {
        return Constants.LIMELIGHT_TO_GOAL_Y_DELTA_METER / Math.sin(Math.toRadians(getYAngle()));
    }

    public Translation2d getVisionPose(){
        
        double distance = getDistanceMetersVision();
        Rotation2d angle = GyroSubsystem.getInstance().getGyroscopeRotation().minus(Rotation2d.fromDegrees(getXAngle()));

        double deltaX = distance * angle.getCos();
        double deltaY = distance *angle.getSin();
        
        Translation2d pose = Constants.GOAL_LOCATION.plus(new Translation2d(deltaX, deltaY));

        return pose;
    }

}
