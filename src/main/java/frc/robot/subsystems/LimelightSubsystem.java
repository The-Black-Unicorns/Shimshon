// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  
  private static LimelightSubsystem limelightSubsystemInstance;

  private NetworkTableEntry tvEntry; //Valid targets
  private NetworkTableEntry txEntry; //X angle
  private NetworkTableEntry tyEntry; //Y angle
  private NetworkTableEntry tlEntry; //Latency
  
  private LimelightSubsystem() {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    tvEntry = limelightTable.getEntry("tv");
    txEntry = limelightTable.getEntry("tx");
    tyEntry = limelightTable.getEntry("ty");
    tlEntry = limelightTable.getEntry("tl");
  }

  public static LimelightSubsystem getInstance(){
    if (limelightSubsystemInstance == null)
      limelightSubsystemInstance = new LimelightSubsystem();

    return limelightSubsystemInstance;
  }

  public boolean hasTarget(){
    if (tvEntry.getDouble(0) == 0){
      return false;
    } else {
      return true;
    }
  } 

  public double getXAngle(){
    return txEntry.getDouble(0);
  }

  @Override
  public void periodic() {
  }
}
