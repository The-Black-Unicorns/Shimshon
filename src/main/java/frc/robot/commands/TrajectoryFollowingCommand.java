// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryFollowingCommand extends CommandBase {

  TrajectoryConfig config;
  PathPlannerTrajectory trajectory;
  HolonomicDriveController controller;

  DrivetrainSubsystem driveTrain;
  //Frames
  int counter = 0;
  //In frames
  int trajectoryLength;
  
  public TrajectoryFollowingCommand(DrivetrainSubsystem drivetrainSubsystem, String trajectoryName, double speedMultiplier) {
    driveTrain = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);


    //Create Trajectory
    // config = new TrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 0.8, DrivetrainSubsystem.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    // config.setKinematics(drivetrainSubsystem.kinematics);
    trajectory = PathPlanner.loadPath(trajectoryName, DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * speedMultiplier, 3);
    trajectoryLength = (int)(trajectory.getTotalTimeSeconds() * 50);
        
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    double kp = 0.9375;
    double kd = 0;

    controller = new HolonomicDriveController(new PIDController(kp, kd, 0),
                                              new PIDController(kp, kd, 0),
                                              new ProfiledPIDController(Constants.ROTATION_FOLLOWING_KP, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 1, Math.PI * 4)));


    System.out.println("Called! Length: " + trajectoryLength);
    counter = 0;

    PathPlannerState headingGoal = (PathPlannerState) trajectory.sample(0);
    Pose2d startingPose = new Pose2d(headingGoal.poseMeters.getX(), headingGoal.poseMeters.getY(), headingGoal.holonomicRotation);
    
    driveTrain.zeroPosition(startingPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      Trajectory.State goal = trajectory.sample(counter * 0.02);
      PathPlannerState headingGoal = (PathPlannerState) trajectory.sample(counter * 0.02);

      ChassisSpeeds movement = controller.calculate(driveTrain.robotPose, goal, Rotation2d.fromDegrees(headingGoal.holonomicRotation.getDegrees()));
      //System.out.println(movement);
      driveTrain.drive(movement);

      counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= trajectoryLength;
  }
}
