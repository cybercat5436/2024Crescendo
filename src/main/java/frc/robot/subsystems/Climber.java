// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
 private TalonFX climberUpper = new TalonFX(Constants.RoboRioPortConfig.CLIMBER_UPPER, "rio");
 private TalonFX climberLower = new TalonFX(Constants.RoboRioPortConfig.CLIMBER_LOWER, "rio");
 

 private Double upperEncoderValue;
 private Double lowerEncoderValue;
 private DutyCycleOut upperClimberRequest;
 private DutyCycleOut lowerClimberRequest;
 
  /** Creates a new Climber. */
  public Climber() {
   // climberUpper.clearStickyFaults();
    //climberLower.clearStickyFaults();

    //climberUpper.restoreFactoryDefaults();
    //climberLower.restoreFactoryDefaults();

    //climberUpper.configStator
    //climberLower.setSmartCurrentLimit(50, 50);
    
    //climberUpper.setNeutralMode(NeutralModeValue.Brake);
    //climberLower.setNeutralMode(NeutralModeValue.Brake);
   this.upperClimberRequest = new DutyCycleOut(0.0);
   this.lowerClimberRequest = new DutyCycleOut(0.0);
    
  }

public void upperClimberControl(double speed){
  //System.out.println("upper climber climbing");
  upperClimberRequest.Output = speed;
  this.climberUpper.setControl(upperClimberRequest);
  System.out.println(speed);
}
public void lowerClimberControl(double speed){
  //System.out.println("lower climber");
  lowerClimberRequest.Output = speed; 
  this.climberLower.setControl(lowerClimberRequest);
  System.out.println(speed);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    upperEncoderValue = climberUpper.getPosition().getValueAsDouble();
    lowerEncoderValue = climberLower.getPosition().getValueAsDouble();

   // upperClimberControl(0);
   // lowerClimberControl(0);

  }

}
