// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Amp extends SubsystemBase {
  /** Creates a new Amp. */
  private TalonFX ampMotor;
  private double speed = 0.5;
  public Amp() {

    ampMotor = new TalonFX(Constants.RoboRioPortConfig.AMP_FALCON_MOTOR);
  }

  public void up() {
    ampMotor.set(ControlMode.PercentOutput,speed);
  }

  public void down() {
    ampMotor.set(ControlMode.PercentOutput,-speed);
  }

  public void stop() {
    ampMotor.set(ControlMode.PercentOutput,0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
