// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class NoteDetector extends SubsystemBase {
  private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
  private double redThreshold = 350;
  private double distanceThreshold = 150; 
  private boolean isNoteDetected = false; 
  private boolean isRumbleActive = false;
  private int successiveReadings = 0; 
  private int successiveReadingsThreshold = 3;
  private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  private NetworkTableEntry redEntry = networkTableInstance.getEntry("colorsensor/redValue");
  private NetworkTableEntry distanceEntry = networkTableInstance.getEntry("colorsensor/distanceValue");
  private CommandXboxController primaryController;
  private CommandXboxController secondaryController;

  private Timer timer;
  private double redValue;
  private double distanceValue;


  /** Creates a new NoteDetector. */
  public NoteDetector(CommandXboxController primaryController, CommandXboxController secondaryController) {
    this.primaryController = primaryController;
    this.secondaryController = secondaryController;
   
    this.timer = new Timer();

    //Register the sendables
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }

  public boolean getIsNoteDetected(){
    return isNoteDetected;
  }

  public void manageRumble(){
    // System.out.println("Manage Rumble");
    if(isNoteDetected && !isRumbleActive){
      timer.reset();
      timer.start();
      startRumble(1.0);
      isRumbleActive = true;
      // System.out.println("Notedetected and rumble set");
    }
   
    if(timer.get()> 0.5 && isNoteDetected) {
      startRumble(0.4);
    } 
    if(!isNoteDetected && isRumbleActive) {
      timer.stop();
      stopRumble();
      isRumbleActive = false;
      // System.out.println("Note not detected");  
    }

  }

  public void startRumble(double intensity){
    secondaryController.getHID().setRumble(RumbleType.kBothRumble, intensity);
    primaryController.getHID().setRumble(RumbleType.kBothRumble, intensity);

  }

  public void stopRumble(){
    secondaryController.getHID().setRumble(RumbleType.kBothRumble, 0);
    primaryController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void periodic() {
    redValue = colorSensor.getRed();
    distanceValue = colorSensor.getProximity();
    boolean isNoteDetectedThisCycle = redValue> redThreshold && distanceValue > distanceThreshold; 
    if (isNoteDetectedThisCycle){
      successiveReadings++;
    }else{
      successiveReadings =0;
    }
    isNoteDetected = successiveReadings >= successiveReadingsThreshold; 

    // SmartDashboard.putNumber("RedNetworkTable", redEntry.getInteger(-1));
    // SmartDashboard.putNumber("DistanceNetworkTable", distanceEntry.getInteger(-1));

    if (DriverStation.isTeleopEnabled()){
      manageRumble();
    }

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("Red", () -> redValue, null);
    builder.addDoubleProperty("Distance", () -> distanceValue, null);
    builder.addBooleanProperty("isNoteDetected", () -> isNoteDetected, null);
  }

  
  
}
