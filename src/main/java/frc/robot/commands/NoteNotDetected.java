// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteDetector;

public class NoteNotDetected extends Command {
  NoteDetector noteDetector;
  /** Creates a new NoteDetectorCommand. */
  public NoteNotDetected(NoteDetector noteDetector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.noteDetector = noteDetector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting NoteNotDetected");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Exiting NoteNotDetected with detection status: " + noteDetector.getIsNoteDetected());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !noteDetector.getIsNoteDetected();
  }
}
