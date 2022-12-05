// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

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

    public ArrayList<Pose2d> getVisionPoses(){
        PhotonPipelineResult results = camera.getLatestResult();
        if (results.hasTargets()){
            List<PhotonTrackedTarget> targets = results.getTargets();
            ArrayList<Pose2d> poses = new ArrayList<Pose2d>(targets.size());
            for (PhotonTrackedTarget target : targets){
                Pose3d targetPose = Constants.TAGS_POSES[target.getFiducialId()];
                Transform3d transformToTarget = target.getBestCameraToTarget();
                Pose3d robotPose3d = targetPose.plus(transformToTarget);
                if (Math.abs(robotPose3d.getZ()) >= 0.2){
                    poses.add(new Pose2d(robotPose3d.getX(), robotPose3d.getY(), robotPose3d.getRotation().toRotation2d()));
                }
            }
            return poses;
        } else{
            return null;
        }
    }

    public void takePicture(){
        camera.takeOutputSnapshot();
    }




}
