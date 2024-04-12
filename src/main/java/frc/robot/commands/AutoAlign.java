// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends Command {

  private SwerveSubsystem swerveSubsystem;
  private LimeLight limeLight;
  private double ySpeed = 0.0;
  private double xSpeed = 0.0;
  private double turningSpeed = 0.0;
  private double kRobotY = 0.08;
  private double kRobotX = 0.08;
  private double targetArea = 0.5;
  private double kPTheta = 0.1;
  private ChassisSpeeds chassisSpeeds;
  // private Timer timer;
  private String autonSelected;
  private double targetHeading;


  /** Creates a new AutoAlign. */
  public AutoAlign(SwerveSubsystem swerveSubsystem, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    this.limeLight = limeLight;
    // timer = new Timer();
    System.out.println("Inside AutoAlign");
     
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timer.reset();
    // timer.start();
    autonSelected = Robot.autonSelected;
    System.out.println("The auton selected is: " + autonSelected);
    if(autonSelected.equals("amp")){
      targetHeading = 60;
    }else if(autonSelected.equals("player")){
      targetHeading = -60;
    }
    targetHeading += DriverStation.getAlliance().get()==DriverStation.Alliance.Red ? 180 : 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thetaError = targetHeading - swerveSubsystem.getRotation2d().getDegrees();
    double yError = limeLight.getVisionTargetHorizontalError();
    double xError = targetArea - limeLight.getVisionArea();
    turningSpeed = thetaError * kPTheta;
    xSpeed = xError * kRobotX;
    ySpeed = yError * kRobotY;
    //ySpeed = limeLight.getVisionTargetHorizontalError() * kLimelightHorizontal;
    // ySpeed = 0;

    chassisSpeeds = new ChassisSpeeds(0.0, 0.0, turningSpeed);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
    // System.out.println("inside AutoAlign execute");
    System.out.println("Turning Speed: "+turningSpeed);
    System.out.println("Theta Error: "+thetaError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // timer.stop();
    // swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Math.abs(limeLight.getVisionTargetHorizontalError())<0.5||timer.get()>2.0){
    //   // System.out.println("tx value: "+limeLight.getVisionTargetHorizontalError());
    //   // System.out.println("time: "+timer.get());
    //   return true;
    // }
    return false;
  }
}
