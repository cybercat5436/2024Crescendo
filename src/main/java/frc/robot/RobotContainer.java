// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.rmi.StubNotFoundException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AbsoluteEncoderCalibration;
import frc.robot.commands.AutoClimbCommand;
import frc.robot.commands.ClimberDefaultCommand;
import frc.robot.commands.ManualEncoderCalibration;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Speaker;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public boolean halfSpeed = false;
    private final LimeLight limeLightGrid = new LimeLight("limelight");
    private final LimeLight limeLightOrient = new LimeLight("limelight-orient");
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    // private final Intake intake = new Intake();
    private final CommandXboxController primaryController = new CommandXboxController(1);
    private final CommandXboxController secondaryController = new CommandXboxController(0);
    private SendableChooser<Command> autonChooser = new SendableChooser<>();

    // swerve subsystem must be instantiated before climber
    private final Climber climber = new Climber();
    private final Speaker speaker = new Speaker();
    private final SuperStructure superStructure = new SuperStructure();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -primaryController.getLeftY(),
        () -> -primaryController.getLeftX(),
        () -> -primaryController.getRightX(),
        () -> !primaryController.start().getAsBoolean(),
        () -> primaryController.rightTrigger().getAsBoolean(),
        () -> primaryController.y().getAsBoolean(),
        () -> primaryController.x().getAsBoolean(),
        () -> primaryController.getLeftTriggerAxis(),
        () -> primaryController.getRightTriggerAxis(),
        limeLightGrid));
      // Configure the button bindings
      ManualEncoderCalibration manualEncoderCalibration = new ManualEncoderCalibration(swerveSubsystem);  
      AbsoluteEncoderCalibration absoluteEncoderCalibration = new AbsoluteEncoderCalibration(swerveSubsystem);           

      SmartDashboard.putData(manualEncoderCalibration);
      SmartDashboard.putData(absoluteEncoderCalibration);
      configureButtonBindings();


      SmartDashboard.putData(new InstantCommand(() -> swerveSubsystem.zeroIntegrator()));

 

      // Register Named Commands for Path Planner
      registerNamedCommands();
      // Build an auto chooser. This will use Commands.none() as the default option.
      String defaultAuton = AutoBuilder.getAllAutoNames().isEmpty() ? "" : AutoBuilder.getAllAutoNames().get(0);
      autonChooser = AutoBuilder.buildAutoChooser(defaultAuton);
      SmartDashboard.putData("Auton Chooser", autonChooser);
}
  
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      
      
      //Intake Buttons
      // primaryController.leftBumper().onTrue(new InstantCommand(()->intake.intakeFeedIn()))
      //   .onFalse(new InstantCommand(()->intake.stopIntake()));
      // primaryController.rightBumper().onTrue(new InstantCommand(()->intake.intakeFeedOut()))
      //   .onFalse(new InstantCommand(()->intake.stopIntake()));
      

      //Climber Bindings

      climber.setDefaultCommand(
        new ClimberDefaultCommand(climber, 
          ()->secondaryController.getLeftY(), 
          ()->secondaryController.getRightY()
        )
      );
      // secondaryController.start().onTrue(
      //    // new ClimberDefaultCommand(climber, ()->secondaryController.getLeftY(), ()->secondaryController.getRightY()
      //    new AutoClimbCommand(climber, swerveSubsystem)
      // );
       secondaryController.x().whileTrue((new AutoClimbCommand(climber, swerveSubsystem, ()-> secondaryController.getLeftY())).repeatedly())
       .onFalse(new InstantCommand(()-> climber.climberStop()));
      //  secondaryController.x().whileTrue(new InstantCommand(() -> System.out.println("Starting AutoClimb...")).repeatedly())
      // .onFalse(new InstantCommand(()-> System.out.println("Exiting AutoClimb...")));

          


      // Speaker Bindings
      Trigger rightTrigger = new Trigger(()->secondaryController.getRightTriggerAxis()> 0.2);
      Trigger leftTrigger = new Trigger(()->secondaryController.getLeftTriggerAxis()> 0.2);
      leftTrigger.whileTrue(new InstantCommand(()-> speaker.startLauncher(secondaryController.getLeftTriggerAxis())).repeatedly())
        
        .onFalse(new InstantCommand(()-> speaker.stopLauncher()));

      rightTrigger.onTrue(new InstantCommand(()-> speaker.startFeeder())).onFalse(new InstantCommand(()-> speaker.stopFeeder()));
      
      // SuperStructure bindings
      secondaryController.y().onTrue(new InstantCommand(()->superStructure.rotateToAmp()));
      secondaryController.a().onTrue(new InstantCommand(()->superStructure.rotateToSpeaker()));
      
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

      return autonChooser.getSelected();
              
  }

  private void registerNamedCommands(){
 
  }

  public SwerveSubsystem getSwerveSubsystem(){
    return swerveSubsystem;
  }

}
  