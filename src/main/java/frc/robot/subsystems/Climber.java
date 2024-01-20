// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
 private CANSparkMax climberUpper = new CANSparkMax(Constants.RoboRioPortConfig.CLIMBER_UPPER, MotorType.kBrushless);
 private CANSparkMax climberLower = new CANSparkMax(Constants.RoboRioPortConfig.CLIMBER_LOWER, MotorType.kBrushless);
 
 private final RelativeEncoder upperEncoder;
 private final RelativeEncoder lowerEncoder;
 
 
  /** Creates a new Climber. */
  public Climber() {
    climberUpper.clearFaults();
    climberLower.clearFaults();

    climberUpper.restoreFactoryDefaults();
    climberLower.restoreFactoryDefaults();

    climberUpper.setSmartCurrentLimit(50, 50);
    climberLower.setSmartCurrentLimit(50, 50);
    
    climberUpper.setIdleMode(IdleMode.kBrake);
    climberLower.setIdleMode(IdleMode.kBrake);

    upperEncoder = climberUpper.getEncoder();
    lowerEncoder = climberLower.getEncoder();
  }

public void upperClimberControl(double speed){
  this.climberUpper.set(speed);
}
public void lowerClimberControl(double speed){
  this.climberLower.set(speed);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
