// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteDetector extends SubsystemBase {
  private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
  private double redThreshold = 350 ;
  private double distanceThreshold = 150 ; 
  private boolean isNoteDetected = false; 
  private int successiveReadings = 0; 
  private int successiveReadingsThreshold = 3;
  private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  private NetworkTableEntry redEntry = networkTableInstance.getEntry("colorsensor/redValue");
  private NetworkTableEntry distanceEntry = networkTableInstance.getEntry("colorsensor/distanceValue");
  
  /** Creates a new NoteDetector. */
  public NoteDetector() {}

  public boolean getIsNoteDetected(){
    return isNoteDetected;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double redValue = colorSensor.getRed();
    double distanceValue = colorSensor.getProximity();
    SmartDashboard.putNumber("Red", redValue);
    SmartDashboard.putNumber("Distance",distanceValue);
    boolean isNoteDetectedThisCycle = redValue> redThreshold && distanceValue > distanceThreshold; 
    if (isNoteDetectedThisCycle){
      successiveReadings++;
    }else{
      successiveReadings =0;
    }
    isNoteDetected = successiveReadings >= successiveReadingsThreshold; 
    SmartDashboard.putBoolean("isNoteDetected", isNoteDetected);

    SmartDashboard.putNumber("RedNetworkTable", redEntry.getInteger(-1));
    SmartDashboard.putNumber("DistanceNetworkTable", distanceEntry.getInteger(-1));

  
  }
}
