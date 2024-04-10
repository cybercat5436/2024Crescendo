// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends Command {

  private SwerveSubsystem swerveSubsystem;
  private LimeLight limeLight;
  private double ySpeed = 0.0;
  private double xSpeed = 0.0;
  private double turningSpeed = 0.0;
  private double kLimelightHorizontal = 0.08;
  private double yScaleFactor = (DriveConstants.ykTranslateDriveMaxSpeedMetersPerSecond/DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond);
  private ChassisSpeeds chassisSpeeds;
  private Timer timer;


  /** Creates a new AutoAlign. */
  public AutoAlign(SwerveSubsystem swerveSubsystem, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.limeLight = limeLight;
    timer = new Timer();
    System.out.println("Inside AutoAlign");
     
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ySpeed = limeLight.getVisionTargetHorizontalError() * kLimelightHorizontal;


    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    chassisSpeeds.vyMetersPerSecond *= yScaleFactor;
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
    System.out.println("inside execute");
    System.out.println("The auton selected is: " + Robot.autonSelected);
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(limeLight.getVisionTargetHorizontalError())<0.5||timer.get()>2.0){
      System.out.println("tx value: "+limeLight.getVisionTargetHorizontalError());
      System.out.println("time: "+timer.get());
      return true;
    }
    return false;
  }
}
