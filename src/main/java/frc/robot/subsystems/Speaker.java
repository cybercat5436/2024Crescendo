// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Speaker extends SubsystemBase {
  /** Creates a new Speaker. */
  private final TalonFX launcher;
  private final CANSparkMax feeder;
  private final VelocityVoltage launcherControl = new VelocityVoltage(0);

  // private TalonFX falconMotor;

  public Speaker() {

    launcher = new TalonFX(Constants.RoboRioPortConfig.SPEAKER_LAUNCHER);
    feeder = new CANSparkMax(Constants.RoboRioPortConfig.SPEAKER_NEO_MOTOR2, MotorType.kBrushless);
    // robot init, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0.48;
    slot0Configs.kD = 0.01;
    launcher.getConfigurator().apply(slot0Configs, 0.050);
    // falconMotor = new TalonFX(Constants.RoboRioPortConfig.SPEAKER_FALCON_MOTOR);

  }
  public void startLauncher() {
    // periodic, run velocity control with slot 0 configs,
    // target velocity of 50 rps
    launcherControl.Slot = 0;
    launcher.setControl(launcherControl.withVelocity(50));
    System.out.println("start launcher");
    // falconMotor.set(ControlMode.PercentOutput,0.5);
  }
public void startLauncher(double percentage) {
    // periodic, run velocity control with slot 0 configs,
    // target velocity of 50 rps
    launcherControl.Slot = 0;
    launcher.setControl(launcherControl.withVelocity(70 *percentage));
    System.out.println("start launcher with percentage " + percentage); 
    // falconMotor.set(ControlMode.PercentOutput,0.5); 
  }

  public void stopLauncher() {
    launcher.set(0);
    System.out.println("stop launcher");
    // falconMotor.set(ControlMode.PercentOutput,0);
  }
  public void startFeeder(){
    feeder.set(1);
    System.out.println("start feeeder");
  }

  public void stopFeeder(){
    feeder.set(0);
    System.out.println("stop feeder");
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putNumber("MotorSpeed",launcher.getVelocity().getValueAsDouble());
  }
}
