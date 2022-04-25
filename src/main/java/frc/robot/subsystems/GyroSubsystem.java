// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GyroSubsystem extends SubsystemBase {

  private final PigeonIMU m_pigeon = new PigeonIMU(Constants.DRIVETRAIN_PIGEON_ID);

  private Rotation2d gyroAngle;
  private Rotation2d gyroOffset = new Rotation2d();

  public GyroSubsystem() {
    // Pigeon status frames
    m_pigeon.setStatusFramePeriod(6, 10);
    m_pigeon.setStatusFramePeriod(11, 10);
    m_pigeon.setStatusFramePeriod(4, 10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private Rotation2d getGyroRotationRaw() {
    return Rotation2d.fromDegrees(m_pigeon.getFusedHeading() * 1.00278552);
  }

  public void updateGyroAngle() {
    gyroAngle = getGyroRotationRaw().minus(gyroOffset);
  }

  public Rotation2d getGyroscopeRotation() {
    return gyroAngle;
}

  public void zeroGyro (Rotation2d newRotation){
    gyroOffset =  getGyroRotationRaw().minus(newRotation);
  }

  public void zeroGyro (){
    zeroGyro(new Rotation2d());
  }

}
