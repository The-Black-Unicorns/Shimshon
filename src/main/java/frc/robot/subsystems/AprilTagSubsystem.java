// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagSubsystem {

    private static AprilTagSubsystem aprilTagSubsystemInstance;
    private AprilTagSubsystem() {
    }
    public static AprilTagSubsystem getInstance() {
        if (aprilTagSubsystemInstance == null)
            aprilTagSubsystemInstance = new AprilTagSubsystem();

        return aprilTagSubsystemInstance;
    }
    
    PhotonCamera camera = new PhotonCamera("camera");

    public void periodic(){
        PhotonPipelineResult results = camera.getLatestResult();
        if (results.hasTargets()){
            List<PhotonTrackedTarget> targets = results.getTargets();
            
        }
    }




}
