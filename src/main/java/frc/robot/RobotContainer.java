// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoClimbCommand;
import frc.robot.commands.AutonResetGyro;
import frc.robot.commands.ClimberDefaultCommand;
import frc.robot.commands.NoteDetectorCommand;
import frc.robot.commands.NoteNotDetected;
import frc.robot.commands.RaceTest;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.NoteDetector;
import frc.robot.subsystems.PoseUpdater;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    private final LimeLight limeLightFront = new LimeLight("limelight-front");
    private final LimeLight limeLightRear = new LimeLight("limelight-rear");
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Intake intake = new Intake();
    private final PoseUpdater poseUpdater = new PoseUpdater(limeLightFront, swerveSubsystem);
    private final CommandXboxController primaryController = new CommandXboxController(1);
    private final CommandXboxController secondaryController = new CommandXboxController(0);
    private final NoteDetector noteDetector = new NoteDetector(primaryController, secondaryController);
    private SendableChooser<Command> autonChooser = new SendableChooser<>();

    // swerve subsystem must be instantiated before climber
    private final SuperStructure superStructure = new SuperStructure();
    // superstructure must be instantiated before climber
    private final Climber climber = new Climber(new InstantCommand(() -> superStructure.rotateToAmp()));
    private final Launcher launcher = new Launcher();
    private final Util util = new Util();
  

    private SequentialCommandGroup shootCommand = new SequentialCommandGroup(
    new InstantCommand(()->launcher.startLauncher(1.0)).repeatedly().withTimeout(.5))
    .andThen(new InstantCommand(()->launcher.startFeeder()).repeatedly().withTimeout(0.3))
    .andThen(new InstantCommand(()->launcher.stop()));
    
    
      
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
      // Configure the button bindings        
      configureButtonBindings();

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
      
      // Intake Buttons;
      secondaryController.b().whileTrue(new InstantCommand(()->intake.intakeFeedIn()).repeatedly())
        .whileFalse(new InstantCommand(()->intake.stopIntake()));
      primaryController.rightBumper().whileTrue(new InstantCommand(()->intake.intakeFeedIn()).repeatedly())
        .whileFalse(new InstantCommand(()->intake.stopIntake()));
      primaryController.leftBumper().onTrue(new InstantCommand(()->intake.intakeFeedOut()))
        .onFalse(new InstantCommand(()->intake.stopIntake()));
      
        


      Trigger primaryRightTrigger = new Trigger(() -> primaryController.getHID().getRightTriggerAxis()> 0.2);
      Trigger primaryStart = new Trigger(()-> primaryController.getHID().getStartButton());
      Trigger primaryYTrigger = new Trigger(() -> primaryController.getHID().getYButton());
      Trigger primaryXTrigger = new Trigger(() -> primaryController.getHID().getXButton());
      

      //Swerve Bindings
      swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -primaryController.getLeftY(),
        () -> -primaryController.getLeftX(),
        () -> -primaryController.getRightX(),
        () -> primaryXTrigger.getAsBoolean(),
        () -> primaryController.getLeftTriggerAxis(),
        () -> primaryController.getRightTriggerAxis(),
        limeLightFront,
        limeLightRear));

      primaryController.back().onTrue(new InstantCommand(()->swerveSubsystem.zeroHeading(0))); //Manually Zero Gyro

      // primaryXTrigger.whileTrue(new AutoAlign(swerveSubsystem, limeLight).repeatedly())
    //.onFalse(new InstantCommand(()->swerveSubsystem.stopModules()));
    // primaryController.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
    //  primaryController.a().onTrue(new Command(primaryController.setRumble(RumbleType.kLeftRumble, 1.0)));
    //  primaryController.a().onTrue(new Command(primaryController.setRumble(RumbleType.kRightRumble, 1.0)));
    // System.out.println("About to rumble");
    
    
    // System.out.println("Rumble started...");

      // Climber bindings
      climber.setDefaultCommand(
        new ClimberDefaultCommand(climber,
          () -> -secondaryController.getLeftY(), 
          () -> -secondaryController.getRightY(),
          () -> secondaryController.getHID().getLeftBumper()
        )
      );


       secondaryController.x().whileTrue((new AutoClimbCommand(climber, swerveSubsystem, ()-> -secondaryController.getLeftY())).repeatedly())
        .onFalse(new InstantCommand(()-> climber.climberStop()));


      // Speaker Bindings
      Trigger rightTrigger = new Trigger(()->secondaryController.getRightTriggerAxis()> 0.2);
      Trigger leftTrigger = new Trigger(()->secondaryController.getLeftTriggerAxis()> 0.2);

      leftTrigger.whileTrue(new InstantCommand(()-> launcher.startLauncher(0.7)).repeatedly())
        .onFalse(new InstantCommand(()-> launcher.stopLauncher()));

      rightTrigger.onTrue(new InstantCommand(()-> launcher.startFeeder()))
        .onFalse(new InstantCommand(()-> launcher.stopFeeder()));
      

      // SuperStructure bindings
      secondaryController.y().onTrue(new InstantCommand(()->superStructure.rotateToAmp()));
      secondaryController.a().onTrue(new InstantCommand(()->superStructure.rotateToSpeaker()));

      // Amp bindings
      secondaryController.rightBumper().whileTrue(new InstantCommand(()->launcher.scoreAmp()).repeatedly())
      .whileFalse(new InstantCommand(()->launcher.stop()));

      // Construct LongShot
      Command longShotCommand = Commands.sequence(
        new InstantCommand(()->{
          System.out.println("<----   Starting Launcher...");
          launcher.startLauncher(1.0);
        }),
        Commands.waitSeconds(1.25),
        new InstantCommand(()->{
          System.out.println("<---  Rotating to LongShot Position...");
          superStructure.rotateToLongShot();
        }),
        Commands.waitSeconds(0.6),
        new InstantCommand(()->{
          System.out.println("<---   Staring Feeder...");
          launcher.startFeeder(1.0);
        }),
        Commands.waitSeconds(0.2),
        new InstantCommand(()->{
          System.out.println("<---   Rotating back down to speaker position...");
          superStructure.rotateToSpeaker();
          launcher.stop();
        }));

      SmartDashboard.putData(longShotCommand);
      // bind longshot to secondary controller
      secondaryController.povUp().onTrue(longShotCommand);

      Command shiftOdometry = new InstantCommand(() -> {
        System.out.println("Shifting y coordinate of odometry providing only pose");
        swerveSubsystem.resetOdometry(
        new Pose2d(swerveSubsystem.getPose().getTranslation().plus(new Translation2d(0.0,0.5)),
         swerveSubsystem.getPose().getRotation()));
        });
      SmartDashboard.putData("Simple Shift", shiftOdometry);

      swerveSubsystem.resetOdometry(new Pose2d(1.0, 2.0, new Rotation2d(45)));
      Command shiftOdometry2 = new InstantCommand(() -> {
        System.out.println("Shifting y coordinate of odometry providing wheel positions and pose");
        poseUpdater.updateOdometry(0.5);
      });
      SmartDashboard.putData("Complex Shift", shiftOdometry2);

      SmartDashboard.putData("Update 0.1", new InstantCommand(() -> poseUpdater.updateOdometry(0.1)));

      // tests of poseUpdater
      System.out.println("ta = 1 => " + poseUpdater.getDistanceEstimate(1));
      System.out.println("ta = 20 => " + poseUpdater.getDistanceEstimate(20));
      System.out.println("ta = 100 => " + poseUpdater.getDistanceEstimate(100));
      System.out.println("tx:19 d:0.5 => " + poseUpdater.calculateYError(19, 0.5));
      System.out.println("tx:8 d:0.25 => " + poseUpdater.calculateYError(8, 0.25));



    
    }
   

    public SwerveSubsystem getSwerveSubsystem(){
      return swerveSubsystem;
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
    NamedCommands.registerCommand("autoAlign", new AutoAlign(swerveSubsystem, limeLightFront));
    NamedCommands.registerCommand("intakeFeedIn", new InstantCommand(()->intake.intakeFeedIn()));
    NamedCommands.registerCommand("stopIntake", new InstantCommand(()->intake.stopIntake()));
    NamedCommands.registerCommand("startLauncher", new InstantCommand(()->launcher.startLauncher(0.7)).repeatedly());
    NamedCommands.registerCommand("startFeeder", new InstantCommand(()->launcher.startFeeder()).repeatedly().withTimeout(0.2));
    NamedCommands.registerCommand("stopShooter", new InstantCommand(()->launcher.stop()));
    NamedCommands.registerCommand("shoot", shootCommand);
    NamedCommands.registerCommand("leftAutonGyroReset", new InstantCommand(()->swerveSubsystem.zeroHeading(60)));
    NamedCommands.registerCommand("rightAutonGyroReset", new InstantCommand(()->swerveSubsystem.zeroHeading(-60)));
    NamedCommands.registerCommand("zeroGyroReset", new InstantCommand(()->swerveSubsystem.zeroHeading(0)));
    NamedCommands.registerCommand("resetGyro", new AutonResetGyro(swerveSubsystem));
    NamedCommands.registerCommand("raceTest", new RaceTest());
    NamedCommands.registerCommand("LivePathTest", swerveSubsystem.testCommand());
    NamedCommands.registerCommand("GetCenterPath", util.getPath(new Pose2d(3.5,5.5,Rotation2d.fromDegrees(0.0))));
    NamedCommands.registerCommand("GetCentertoSpeaker", util.getPath(new Pose2d(1.40,5.5,Rotation2d.fromDegrees(0.0))));
    NamedCommands.registerCommand("NoteDetector", new NoteDetectorCommand(noteDetector));
    NamedCommands.registerCommand("detectNote", new NoteDetectorCommand(noteDetector));
    NamedCommands.registerCommand("returnToCenterSubWoofer", swerveSubsystem.returnToCenterSubWoofer());
    NamedCommands.registerCommand("NoteNotDetected", new NoteNotDetected(noteDetector));
    NamedCommands.registerCommand("disablePoseUpdater", new InstantCommand(() -> poseUpdater.isEnabled = false));
    NamedCommands.registerCommand("enablePoseUpdater", new InstantCommand(() -> poseUpdater.isEnabled = true));
  }

  /**
   * Looks at the auton command selected in auton chooser.  If no starting pose is present, it sets the robot's starting pose
   * to the initial pose of the first path
   * @param autonName:  name of auton path selected
   */
  public void setStartingPoseIfMissing(String autonName){
    System.out.println("Selected Auton: " + autonName);
    String poseString = "Unknown";
    
    try {

      poseString = PathPlannerAuto.getStaringPoseFromAutoFile(autonName).toString();
      System.out.println("StartingPose from Auto: " + poseString);

      List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autonName);
      System.out.println("First Position + flipped: " + getFirstPosition(paths).toString());


    } catch (RuntimeException e){
      // Exception thrown if starting pose is null in PathPlanner Auton file
      System.out.println("No starting pose detected in Auton file!");
      
      // get a list of all paths present in the auton file
      List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autonName);

      // figure out if path flipping has to happen
      boolean shouldFlip = false;
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if(alliance.isPresent()){
        if(alliance.get() == DriverStation.Alliance.Red){
          shouldFlip = true;
        }
      }

      // make sure the list isn't empty
      if(!paths.isEmpty()){
        PathPlannerPath firstPath = paths.get(0);
        if (shouldFlip) firstPath.flipPath();
        Pose2d initialPose = firstPath.getPreviewStartingHolonomicPose();
        System.out.println("Starting pose from first path: " + initialPose);
        System.out.println("Setting robot pose...");
        swerveSubsystem.resetOdometry(initialPose);
        poseString = initialPose.toString();
      } else{
        // no pose provided and no paths present
        System.out.println("Initial Pose is unknown, potential issue!!!");
        // throw new RuntimeException("No Initial Pose for the robot provided");
      }
    }

    System.out.println("Exiting setStartingPoseIfMissing with robot pose: " + poseString);
  }

  private Translation2d getFirstPosition(List<PathPlannerPath> paths){

      // figure out if path flipping has to happen
      boolean shouldFlip = false;
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if(alliance.isPresent()){
        if(alliance.get() == DriverStation.Alliance.Red){
          shouldFlip = true;
        }
      }

      Translation2d startPosition = new Translation2d();
        // make sure the list isn't empty
      if(!paths.isEmpty()){
        PathPlannerPath firstPath = paths.get(0);
        if (shouldFlip) {
          firstPath = firstPath.flipPath();
        }
        startPosition = firstPath.getAllPathPoints().get(0).position;
      }

      return startPosition;
    }

}
  