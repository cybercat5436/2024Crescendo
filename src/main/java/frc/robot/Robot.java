// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static Robot robot;

  public static String autonSelected = "amp";



  private Timer timer = new Timer();

  private NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private NetworkTableEntry redEntry = networkTable.getEntry("colorsensor/redValue");
  private NetworkTableEntry distanceEntry = networkTable.getEntry("colorsensor/distanceValue");
  private int count = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robot = this;
    m_robotContainer = new RobotContainer();
    // timer.start();
    // addPeriodic(()->{
    //   m_robotContainer.getSwerveSubsystem().getOdometry().update(m_robotContainer.getSwerveSubsystem().getRotation2d(), m_robotContainer.getSwerveSubsystem().getModulePositions());    
    // }, 
    //   0.01, 0.0);
  }

 

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    distanceEntry.setInteger(1000 + count);
    redEntry.setInteger(count++);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // check if starting pose is provided
    m_robotContainer.setStartingPoseIfMissing(m_autonomousCommand.getName());

    //Check if the string contains "player" in the selected autonomous(ignores case)
     if (m_autonomousCommand.getName().toLowerCase().contains("player")){
       autonSelected = "player";
     }
    //Check if the string contains "amp" in the selected autonomous(ignores case)
     if (m_autonomousCommand.getName().toLowerCase().contains("amp")){
       autonSelected = "amp";
     }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    double headingDegrees = m_robotContainer.getSwerveSubsystem().getOdometry().getPoseMeters().getRotation().getDegrees();
    System.out.println(headingDegrees);
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get()==DriverStation.Alliance.Red){
        headingDegrees+=180;
      }
    }
    m_robotContainer.getSwerveSubsystem().zeroHeading(headingDegrees);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
