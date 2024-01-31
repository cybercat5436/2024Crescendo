// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Climber;

public class ClimberDefaultCommand extends Command {
  private DoubleSupplier upperSpeedFunction, lowerSpeedFunction;
  private Climber climber;
  /** Creates a new ClimberDefaultCommand. */
  public ClimberDefaultCommand(Climber climber, DoubleSupplier upperSpeedFunction, DoubleSupplier lowerSpeedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    this.upperSpeedFunction = upperSpeedFunction;
    this.lowerSpeedFunction = lowerSpeedFunction;
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Climber is climbing");
      climber.upperClimberControl(upperSpeedFunction.getAsDouble());
      climber.lowerClimberControl(lowerSpeedFunction.getAsDouble());
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
