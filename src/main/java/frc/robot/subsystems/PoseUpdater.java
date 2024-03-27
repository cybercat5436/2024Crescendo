// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseUpdater extends SubsystemBase {
  /** Creates a new PoseUpdater. */
  private final LimeLight limeLightFront;
  private final SwerveSubsystem swerveSubsystem;
  private double tx;
  private double ta;
  private boolean isTargetVisible;
  public NetworkTableEntry txLocal;
  public NetworkTableEntry tyLocal;
  public NetworkTableEntry taLocal;

  public PoseUpdater(LimeLight limeLightFront, SwerveSubsystem swerveSubsystem) {
    this.limeLightFront = limeLightFront;
    this.swerveSubsystem = swerveSubsystem;
    txLocal = limeLightFront.txLocal;
    taLocal = limeLightFront.taLocal;
  }
  public double getDistanceEstimate(double ta) {
  // 1m: Ta = 4.2
  // .75m: Ta= 5.8-6.2
  // .5m: Ta = 12-16
  // .25m: Ta = 28-36
    return -0.013*ta + .682; //equation from testing
  }
  public double getYError(double tx, double distanceEstimate) {
  // Side(left = +, right = -)
  // 24Cm side, .5m away: Tx = 19
  // 18Cm side, .25m away: Tx = 8
    double slope = -3.96*distanceEstimate + 3.2;
    return (slope*tx)/100;
  }
  public void updateOdometry(double yError) {
    // Transform2d transform2d = new Transform2d(new Translation2d(0,yError),new Rotation2d());
    Translation2d translationAdjustments = new Translation2d(0,yError);
    Translation2d translation2d = swerveSubsystem.getOdometry().getPoseMeters().getTranslation().plus(translationAdjustments);
    Pose2d pose2d = new Pose2d(translation2d,new Rotation2d());
    swerveSubsystem.getOdometry().resetPosition(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions(), pose2d);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isTargetVisible = limeLightFront.getVisionTargetStatus();
    if(isTargetVisible) {
      ta = taLocal.getDouble(0);
      tx = txLocal.getDouble(0);
      double distance = getDistanceEstimate(ta);
      double yError = getYError(tx, distance);
      SmartDashboard.putNumber("Y Error", yError);
      SmartDashboard.putNumber("Distance Estimator", distance);
      updateOdometry(yError);
      SmartDashboard.putString("Odometry String", swerveSubsystem.getOdometry().toString());
    }else{
      ta = 0;
      tx = 0;
    }
  }
}
