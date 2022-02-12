// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryFollowingCommand extends CommandBase {

  TrajectoryConfig config;
  PathPlannerTrajectory trajectory;
  HolonomicDriveController controller;
  Joystick taranis;

  DrivetrainSubsystem driveTrain;
  //Frames
  int counter = 0;
  //In frames
  int trajectoryLength;
  
  public TrajectoryFollowingCommand(DrivetrainSubsystem drivetrainSubsystem, Joystick taraniscController) {
    driveTrain = drivetrainSubsystem;
    taranis = taraniscController;
    addRequirements(drivetrainSubsystem);


    //Create Trajectory
    config = new TrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 0.2, DrivetrainSubsystem.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    config.setKinematics(drivetrainSubsystem.kinematics);
    // Pose2d initialPose = new Pose2d();
    // Pose2d endPose = new Pose2d(2, 0, new Rotation2d(0));
    // ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();
    // waypoints.add(new Translation2d(0.75, -0.3));
    // waypoints.add(new Translation2d(1.5, 0.3));

    // trajectory = TrajectoryGenerator.generateTrajectory(initialPose, waypoints, endPose, config);
    trajectory = PathPlanner.loadPath("Test path", DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 0.2, DrivetrainSubsystem.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    trajectoryLength = (int)(trajectory.getTotalTimeSeconds() * 50);
    //Create holonomic controller
        
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    double kp = 1.5 + taranis.getRawAxis(4);
    SmartDashboard.putNumber("Kp", kp);
    kp = 0.9375;
    double kd = 0.5 + taranis.getRawAxis(5) * 0.5;
    SmartDashboard.putNumber("Kd", kd);
    kd = 0;

    controller = new HolonomicDriveController(new PIDController(kp, kd, 0),
                                              new PIDController(kp, kd, 0),
                                              new ProfiledPIDController(Constants.ROTATION_FOLLOWING_KP, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 1, Math.PI * 4)));


    System.out.println("Called! Length: " + trajectoryLength);
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      Trajectory.State goal = trajectory.sample(counter * 0.02);
      SmartDashboard.putNumber("Trajectory Speed", goal.velocityMetersPerSecond);
      PathPlannerState headingGoal = (PathPlannerState) trajectory.sample(counter * 0.02);

      ChassisSpeeds movement = controller.calculate(driveTrain.robotPose, goal, Rotation2d.fromDegrees(headingGoal.holonomicRotation.getDegrees()));
      SmartDashboard.putNumber("Controller Speed", movement.vxMetersPerSecond);
      //System.out.println(movement);
      driveTrain.drive(movement);

      counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= trajectoryLength;
  }
}
