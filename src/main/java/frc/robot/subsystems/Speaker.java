// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Speaker extends SubsystemBase {
  /** Creates a new Speaker. */
  private final CANSparkMax neoMotor1;
  private final CANSparkMax neoMotor2;
  private TalonFX falconMotor;

  public Speaker() {

    neoMotor1 = new CANSparkMax(Constants.RoboRioPortConfig.SPEAKER_NEO_MOTOR1, MotorType.kBrushless);
    neoMotor2 = new CANSparkMax(Constants.RoboRioPortConfig.SPEAKER_NEO_MOTOR2, MotorType.kBrushless);
    falconMotor = new TalonFX(Constants.RoboRioPortConfig.SPEAKER_FALCON_MOTOR);

  }
  public void start() {
    neoMotor1.set(1);
    neoMotor2.set(1);
    falconMotor.set(ControlMode.PercentOutput,0.5);

  }
  public void stop() {
    neoMotor1.set(0);
    neoMotor2.set(0);
    falconMotor.set(ControlMode.PercentOutput,0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
