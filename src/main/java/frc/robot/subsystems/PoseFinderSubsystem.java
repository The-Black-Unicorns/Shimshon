// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

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

    private SwerveDrivePoseEstimator poseEstimator;
    private Pose2d robotPose;
    PhotonCamera camera = new PhotonCamera("gloworm");
    private double latency;

    public void initialize (SwerveDriveKinematics kinematics){
        poseEstimator = new SwerveDrivePoseEstimator(GyroSubsystem.getInstance().getGyroscopeRotation(), new Pose2d(),
        kinematics,
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(Units.degreesToRadians(0.01)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    }

    public Pose2d addOdometry(SwerveModuleState[] states){
        poseEstimator.update(GyroSubsystem.getInstance().getGyroscopeRotation(), states[0], states[1], states[2], states[3]);
        return robotPose;
    }

    public void addVisionData(){
        PhotonPipelineResult results = camera.getLatestResult();
        latency = results.getLatencyMillis();
        System.out.println(results.hasTargets());
        if (results.hasTargets()){
            PhotonTrackedTarget target = results.getBestTarget();
            Pose3d targetPose = Constants.TAGS_POSES[target.getFiducialId()];
            Transform3d transformToTarget = target.getBestCameraToTarget();
            Pose3d robotPose3d = targetPose.transformBy(transformToTarget.inverse());
            // System.out.println(robotPose3d.toPose2d());
            poseEstimator.addVisionMeasurement(robotPose3d.toPose2d(), Timer.getFPGATimestamp() - latency / 1000);
        }
    }


    public void setPosition(Translation2d location){
        poseEstimator.resetPosition(new Pose2d(location, GyroSubsystem.getInstance().getGyroscopeRotation()), GyroSubsystem.getInstance().getGyroscopeRotation());
    }

    public void setPose(Pose2d pose){
        GyroSubsystem.getInstance().zeroGyro(pose.getRotation());
        poseEstimator.resetPosition(pose, pose.getRotation());
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public double getLatencyMillis(){
        return latency;
    }

    public void takePicture(){
        camera.takeOutputSnapshot();
    }

}
