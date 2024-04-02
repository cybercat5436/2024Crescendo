// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class AprilTagVision extends SubsystemBase {

public boolean isEnabled = false;

public LimeLight limeLightRear;


  /** Creates a new AprilTagVision. */
  public AprilTagVision(LimeLight limeLightRear) {
    this.limeLightRear = limeLightRear;
  }

  public PoseEstimate updatePoseEstimator(SwerveDrivePoseEstimator poseEstimator){
    // adds vision targets to provided pose estimator if active
    if (!isEnabled) return null;

    // Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still accounting for measurement noise.
    // This method can be called as infrequently as you want, as long as you are calling update(edu.wpi.first.math.geometry.Rotation2d, 
    //   edu.wpi.first.math.kinematics.SwerveModulePosition[]) every loop.
    // To promote stability of the pose estimate and make it robust to bad vision data, we recommend only adding vision 
    //   measurements that are already within one meter or so of the current pose estimate.

    // 1) get the pose estimate from rear limelight
    // 2) see if it is within 1m of odometry pose
    // 3) if so, add it as a vision measurement
    
    // 1) get the pose estimate from rear limelight
    PoseEstimate visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLightRear.networkTableName);
    
    // only continue if target is visible and poseEstimate is not null
    if(!limeLightRear.getVisionTargetStatus() || visionPoseEstimate == null) return null;
    
    // 2) see if it is within 1m of odometry pose
    double offsetDistFromOdometry = poseEstimator.getEstimatedPosition().getTranslation().getDistance(visionPoseEstimate.pose.getTranslation());
    // don't include vision target if more than 1m off
    if(offsetDistFromOdometry > 1) return null;
    
    // 3) if so, add it as a vision measurement
    // From https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#pose-estimation
    poseEstimator.addVisionMeasurement(visionPoseEstimate.pose, visionPoseEstimate.timestampSeconds);

    return visionPoseEstimate;

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
