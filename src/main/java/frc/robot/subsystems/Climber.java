// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
/**
 * positive value raises arms, negative values lowers arms. Encoder values increase when arms raise.
 */
public class Climber extends SubsystemBase {
 private TalonFX rightClimberMotor = new TalonFX(Constants.RoboRioPortConfig.CLIMBER_RIGHT, "rio");
 private TalonFX leftClimberMotor = new TalonFX(Constants.RoboRioPortConfig.CLIMBER_LEFT, "rio");
 

 private Double rightEncoderValue;
 private Double leftEncoderValue;
 private DutyCycleOut rightClimberRequest;
 private DutyCycleOut leftClimberRequest;
 private double climberEncoderLimit = 1;

 
 
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
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentThreshold = 50;
    config.CurrentLimits.SupplyTimeThreshold = 0.3;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
   leftClimberMotor.getConfigurator().apply(config);
   rightClimberMotor.getConfigurator().apply(config);
   this.rightClimberRequest = new DutyCycleOut(0.0);
   this.leftClimberRequest = new DutyCycleOut(0.0);

    
  }

public void moveRightClimber(double speed){
  //System.out.println("upper climber climbing");
  //rightClimberRequest.Output = speed;
  if(rightEncoderValue < climberEncoderLimit && speed < 0){
      speed = 0.0;
  }
  this.rightClimberMotor.setControl(rightClimberRequest.withOutput(speed));
  // System.out.println(speed);

}
public void moveLeftClimber(double speed){
  //System.out.println("lower climber");
  //leftClimberRequest.Output = speed;
  if(leftEncoderValue < climberEncoderLimit && speed < 0){
      speed = 0.0; 
  }
  this.leftClimberMotor.setControl(leftClimberRequest.withOutput(speed));
  // System.out.println(speed);
}
public void climberStop(){
 this.leftClimberMotor.setControl(leftClimberRequest.withOutput(0));
 this.rightClimberMotor.setControl(rightClimberRequest.withOutput(0));

}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightEncoderValue = rightClimberMotor.getPosition().getValueAsDouble();
    leftEncoderValue = leftClimberMotor.getPosition().getValueAsDouble();
   // upperClimberControl(0);
   // lowerClimberControl(0);

  }

}
