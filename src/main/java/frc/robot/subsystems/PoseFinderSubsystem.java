// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class PoseFinderSubsystem {
    //Singelton stuff
    private static PoseFinderSubsystem instance;
    private PoseFinderSubsystem (){
    }
    public static PoseFinderSubsystem getInstance(){
        if (instance == null){
            instance = new PoseFinderSubsystem();
        }
        return instance;
    }

    SwerveDrivePoseEstimator poseEstimator;
    Pose2d robotPose;
    

    public void initialize (SwerveDriveKinematics kinematics){
        poseEstimator = new SwerveDrivePoseEstimator(GyroSubsystem.getInstance().getGyroscopeRotation(), new Pose2d(),
        kinematics,
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(Units.degreesToRadians(0.01)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    }

    public Pose2d addOdometry(SwerveModuleState[] states){


        return robotPose;
    }

    public void setPosition(Transform2d location){

    }

    public void setPose(Pose2d pose){
        
    }

    public Pose2d getPose(){
        return robotPose;
    }

}
