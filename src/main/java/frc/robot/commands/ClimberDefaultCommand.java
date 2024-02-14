// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Climber;

public class ClimberDefaultCommand extends Command {
  private DoubleSupplier rightSpeedFuction, leftSpeedFunction;
  private Climber climber;
  /** Creates a new ClimberDefaultCommand. */
  public ClimberDefaultCommand(Climber climber, DoubleSupplier leftSpeedFunction, DoubleSupplier rightSpeedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    this.rightSpeedFuction = rightSpeedFunction;
    this.leftSpeedFunction = leftSpeedFunction;
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Climber default command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Climber is climbing");
      climber.moveLeftClimber(leftSpeedFunction.getAsDouble()*0.3);
      climber.moveRightClimber(rightSpeedFuction.getAsDouble()*0.3);
     // System.out.println(upperSpeedFunction.getAsDouble());
      //System.out.println(lowerSpeedFunction.getAsDouble());
      
  }
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
