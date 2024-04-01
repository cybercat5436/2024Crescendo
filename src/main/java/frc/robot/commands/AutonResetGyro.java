// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonResetGyro extends Command {
  /** Creates a new AutonResetGyro. */
  private SwerveSubsystem swerveSubsystem;

  public AutonResetGyro(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double headingDegrees = swerveSubsystem.getSwerveDrivePoseEstimator().getEstimatedPosition().getRotation().getDegrees();
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get()==DriverStation.Alliance.Red){
        headingDegrees+=180;
      }
    }
    swerveSubsystem.zeroHeading(headingDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
