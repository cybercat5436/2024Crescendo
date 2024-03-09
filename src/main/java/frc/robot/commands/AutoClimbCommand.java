// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoClimbCommand extends Command {
  private Climber climber;
  private SwerveSubsystem swerveSubsystem;
  private double targetAngle = 0.0;
  private double rollError;
  private double balanceSpeed;
  private double kP = 0.1;
  private DoubleSupplier leftSpeedFunction;
  
  
  /** Creates a new AutoClimbCommand. */
  public AutoClimbCommand(Climber climber, SwerveSubsystem swerveSubsystem, DoubleSupplier leftSpeedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    this.climber = climber;
    this.swerveSubsystem = swerveSubsystem;
    this.leftSpeedFunction = leftSpeedFunction;
    this.targetAngle = swerveSubsystem.getBalanceRoll();

    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double targetAngle = 0.0;
    double rollAngle = swerveSubsystem.getRollDegrees();

    // Wnen looking at robot from the back
    // --CW roll is POSITIVE
    // --CCW roll is NEGATIVE

    // But when calculating error these signs flip
    rollError = this.targetAngle - rollAngle;
    // System.out.println("roll error: " + rollError);
    double climbSpeed = Math.abs(leftSpeedFunction.getAsDouble()) > 0.2 ? 0.6 : 0.0;
    climbSpeed = Math.signum(leftSpeedFunction.getAsDouble()) * climbSpeed;

    balanceSpeed = Math.abs(kP*rollError);
    balanceSpeed = Math.min(balanceSpeed, 0.2);

    double rightSpeed = 0.0;
    double leftSpeed = 0.0;
    
    if(rollError > 0){
      // rollError is POSITIVE if robot is tilted to LEFT (when looking from the back)
      leftSpeed = climbSpeed - balanceSpeed;
      rightSpeed = climbSpeed + balanceSpeed;
    } else if(rollError < 0){
      // rollError is NEGATIVE if robot is tilted to RIGHT (when looking from the back)
      leftSpeed = climbSpeed + balanceSpeed;
      rightSpeed = climbSpeed - balanceSpeed;
    }else{
      leftSpeed = (0);
      rightSpeed = (0);
    }
    climber.moveLeftClimber(leftSpeed);
    climber.moveRightClimber(rightSpeed);
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

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("rollError", () -> this.rollError, null);
    builder.addDoubleProperty("balanceSpeed", () -> this.balanceSpeed, null);
    builder.addDoubleProperty("kP", () -> this.kP, (value) -> this.kP = value);
    builder.addDoubleProperty("Target Roll Angle", () -> this.targetAngle, null);
  }
}
