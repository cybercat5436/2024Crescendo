// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
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

  public double yError = 0.0;
  public double distanceEstimate = 0.0;
  public boolean isEnabled = true;   // can be to prevent updates during certain periods in auton
  public boolean isLockedOut = false;   // odometry updates can be locked out for a period of time after updating
  public int cyclesSinceLocked = 0;
  public final int CYCLE_LOCKOUT = 8;

  public PoseUpdater(LimeLight limeLightFront, SwerveSubsystem swerveSubsystem) {
    this.limeLightFront = limeLightFront;
    this.swerveSubsystem = swerveSubsystem;
    txLocal = limeLightFront.txLocal;
    taLocal = limeLightFront.taLocal;
    cyclesSinceLocked = 0;

    //Register the sendables
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }


  public double getDistanceEstimate(double ta) {
    // 1m: Ta = 4.2
    // .75m: Ta= 5.8-6.2
    // .5m: Ta = 12-16
    // .25m: Ta = 28-36
    double x1=14, x2=32, y1=0.5, y2=0.25;
    double m = (y2-y1) / (x2-x1);  //slope
    double d = (ta-x1) * m + y1;

    // Bound distance 0 <= d <= y1
    d = Math.min(y1, d);  // don't let number exceed y1
    d = Math.max(0, d); // don't let be less than 0
    return d;
    // return -0.013*ta + .682; //equation from testing
  }

  public double calculateYError(double tx, double distanceEstimate) {
    // this calculates and returns yError but doesn't set the instance variable

    // Side(left = +, right = -)
    // 24Cm side, .5m away: Tx = 19
    // 18Cm side, .25m away: Tx = 8
    double s1=24/19, s2=18/8, d1=0.5, d2=0.25;
    double mSens = (s2-s1) / (d2/d1);  // sensitity slope as function of distance
    double sensitivity = mSens * (distanceEstimate - d1);
    // bound sensitivity 1 < s < 2.25
    sensitivity = Math.min(2.25, sensitivity);
    sensitivity = Math.max(1.0, sensitivity);

    double yError = (tx * sensitivity) / 100;
    return yError;
    
    // old formula
    // double slope = -3.96*distanceEstimate + 3.2;
    // return (slope*tx)/100;
  }

  public void updateOdometry(double offsetValue) {
    // Transform2d transform2d = new Transform2d(new Translation2d(0,yError),new Rotation2d());
    Pose2d currentPose = swerveSubsystem.getOdometry().getPoseMeters();
    Translation2d translationAdjustment = new Translation2d(0, offsetValue);
    Translation2d translation2d = currentPose.getTranslation().plus(translationAdjustment);
    
    // These are from Tuesday, when the x coordinate was changing.  Note:  Rotation being set to 0
    Pose2d newPose = new Pose2d(translation2d, new Rotation2d());
    // Pose2d newPose = new Pose2d(translation2d, currentPose.getRotation());
    printInfo(currentPose, newPose);
    swerveSubsystem.getOdometry().resetPosition(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions(), newPose);
    
    // This is proposed method where rotation is preserved from pose
    // Pose2d newPose = new Pose2d(translation2d, currentPose.getRotation());
    // swerveSubsystem.resetOdometry(newPose);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isTargetVisible = limeLightFront.getVisionTargetStatus();
    
    // Calculate error
    if(isTargetVisible) {
      
      calculateYError();

      // update pose if active  
      if (isEnabled && !isLockedOut) {
        updateOdometry(yError);
        // prevent over-eager updating of odometry
        startLockoutPeriod();
      }
    }else{
      ta = 0;
      tx = 0;
      distanceEstimate = 0.0;
      yError = 0.0;
    }

    // manage lockout state
    if(isLockedOut && cyclesSinceLocked > CYCLE_LOCKOUT){
      endLockoutPeriod();
    }

    cyclesSinceLocked++;
  }

  public void startLockoutPeriod() {
    isLockedOut = true;
    cyclesSinceLocked = 0;
  }

  public void endLockoutPeriod(){
    isLockedOut = false;
  }

  public double calculateYError(){
    // this sets yError instance variable
    ta = taLocal.getDouble(0);
    tx = txLocal.getDouble(0);
    distanceEstimate = getDistanceEstimate(ta);
    yError = calculateYError(tx, distanceEstimate);
    return yError;
  }


  private void printInfo(Pose2d currentPose, Pose2d newPose){
    System.out.println(String.format("PoseUpdater:: tx: %.2f ta: %.2f with yError: %.2f and distance: %.2f", tx, ta, yError, distanceEstimate));
    System.out.println(String.format("Updating from: %s\nUpdating to:   %s", currentPose.toString(), newPose.toString()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleArrayProperty("tx-ta", () -> new double[] {tx, ta}, null);
    builder.addDoubleProperty("Y Error", () -> yError, null);
    builder.addDoubleProperty("Distance Estimate", () -> distanceEstimate, null);
    builder.addBooleanProperty("isEnabled", () -> isEnabled, (value) -> isEnabled = value);
  }

}
