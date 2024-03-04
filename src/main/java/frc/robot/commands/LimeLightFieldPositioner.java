// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;;

public class LimeLightFieldPositioner extends Command {
  public CANdle candle = new com.ctre.phoenix.led.CANdle(Constants.RoboRioPortConfig.CANDLE, Constants.RoboRioPortConfig.Canivore);
  private SwerveDrivePoseEstimator poseEstimator;
  LimelightHelpers.PoseEstimate limelightMeasurement;

  /** Creates a new LimeLightFieldPositioner. */
  public LimeLightFieldPositioner(LimeLight limeLightRear, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimator = swerveSubsystem.getPoseEstimator();

    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    this.candle.configAllSettings(configAll, 100);

    //Register the sendables
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }

  @Override
  public boolean runsWhenDisabled() {
    // TODO Auto-generated method stub
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("<****---  INITIALIZING LimelightFieldPositioner  ---****>");

    // Set the CANdle colors based on alliance
    if (DriverStation.getAlliance().isPresent()){
      if (DriverStation.getAlliance().get() == Alliance.Red){
        candle.setLEDs(255, 0, 0, 0, 0, 8);
        return;
      }
    }
    
    candle.setLEDs(0, 0, 255, 0, 0, 8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get the localized position from the LimeLight
        
    this.limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if(limelightMeasurement.tagCount >= 2)
    {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      poseEstimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // DriverStation.isTeleopEnabled();

    System.out.println("<***---  Exiting LimelightFieldPositioner  ---***>");
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean shouldExit = DriverStation.isEnabled();
    return shouldExit;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    
    builder.addStringProperty("Single Tag Estimate", 
      () -> limelightMeasurement == null ? "No Reading" : limelightMeasurement.pose.toString(), null);

    builder.addStringProperty("Multi Tag Pose Est", 
      () -> poseEstimator == null ? "No Reading" : poseEstimator.getEstimatedPosition().toString(), null);
  }
}
