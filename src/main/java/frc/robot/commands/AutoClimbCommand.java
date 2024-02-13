// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoClimbCommand extends Command {
  private Climber climber;
  private SwerveSubsystem swerveSubsystem;
  private double targetAngle = 0.0;
  /** Creates a new AutoClimbCommand. */
  public AutoClimbCommand(Climber climber, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    this.climber = climber;
    this.swerveSubsystem = swerveSubsystem;
    this.targetAngle = swerveSubsystem.getBalanceRoll();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double targetAngle = 0.0;
    double rollAngle = swerveSubsystem.getRollDegrees();
    double rollError = this.targetAngle - rollAngle;
    System.out.println("roll error: " + rollError);
    double kP = 0.05;
    double speed =Math.abs(kP*rollError);
    speed = Math.min(speed, 0.3);
    if(rollError>0){
      climber.moveLeftClimber(speed);
      climber.moveRightClimber(0);
    } else if(rollError<0){
      climber.moveRightClimber(speed);
      climber.moveLeftClimber(0);
    }else{
      climber.moveLeftClimber(0);
      climber.moveRightClimber(0);
    }


  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // climber.climberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
