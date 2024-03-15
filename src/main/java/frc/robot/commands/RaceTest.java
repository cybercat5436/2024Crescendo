// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;

public class RaceTest extends Command {
  /** Creates a new RaceTest. */
  private Timer timer = new Timer();
  public RaceTest() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for(int i=0; i<20; i++){
      System.out.println("Race test ended");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>5.0;
  }
}
