// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Launcher extends SubsystemBase {
  /** Creates a new Speaker. */
  private final TalonFX launcher;
  private final CANSparkMax feeder;
  private final VelocityVoltage launcherControl = new VelocityVoltage(0);

  // private TalonFX falconMotor;

  public Launcher() {

    launcher = new TalonFX(Constants.RoboRioPortConfig.SPEAKER_LAUNCHER, Constants.RoboRioPortConfig.Canivore);
    feeder = new CANSparkMax(Constants.RoboRioPortConfig.SPEAKER_FEEDER_MOTOR, MotorType.kBrushless);
   
    // robot init, set slot 0 gains
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //var slot0Configs = config.Slot0. Slot0Configs();
   // var slot0Configs = new Slot0Configs();
    config.Slot0.kV = 0.12;
    config.Slot0.kP = 0.11;
    config.Slot0.kI = 0.48;
    config.Slot0.kD = 0.01;
    launcher.getConfigurator().apply(config, 0.050);
    // falconMotor = new TalonFX(Constants.RoboRioPortConfig.SPEAKER_FALCON_MOTOR);
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }
  public void startLauncher() {
    // periodic, run velocity control with slot 0 configs,
    // target velocity of 50 rps
    launcherControl.Slot = 0;
    launcher.setControl(launcherControl.withVelocity(50));
    // System.out.println("start launcher");
    // falconMotor.set(ControlMode.PercentOutput,0.5);
  }
public void startLauncher(double percentage) {
    // periodic, run velocity control with slot 0 configs,
    // target velocity of 50 rps
    launcherControl.Slot = 0;
    launcher.setControl(launcherControl.withVelocity(100 *percentage));
    // System.out.println("start launcher with percentage " + percentage); 
    // falconMotor.set(ControlMode.PercentOutput,0.5); 
  }

  public void stopLauncher() {
    launcher.set(0);
    // System.out.println("stop launcher");
    // falconMotor.set(ControlMode.PercentOutput,0);
  }
  public void startFeeder(){
    feeder.set(1);
    // System.out.println("start feeeder");
  }
   public void startFeeder(double speed){
    feeder.set(speed);
    // System.out.println("start feeeder");
  }

  public void stopFeeder(){
    feeder.set(0);
    // System.out.println("stop feeder");
  }
  public void scoreAmp(){
    startLauncher(0.5);
    startFeeder(0.5);
    // System.out.println("start amp");
  }
  public void stop(){
  stopFeeder();
  stopLauncher();
  // System.out.println("stop amp");
  }

  
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("Launcher speed", () -> launcher.getVelocity().getValueAsDouble(), null);
  }
  
}

