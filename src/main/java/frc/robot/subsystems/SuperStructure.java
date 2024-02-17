// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class SuperStructure extends SubsystemBase {
  /** Creates a new SuperStructure. */
  private TalonFX superStructure;
  MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  private double ampPosition = 7;
  // 8.9
  public SuperStructure() {

    superStructure = new TalonFX(Constants.RoboRioPortConfig.SUPERSTRUCTURE_MOTOR);
    // class member variable
    
    // robot init
    var talonFXConfigs = new TalonFXConfiguration();
    // talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 4.8;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 10; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 20; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 200; // 1600 rps/s^2 jerk (0.1 seconds)

    superStructure.getConfigurator().apply(talonFXConfigs, 0.050);

   


  }
  public void rotateToAmp() {

     // periodic, run Motion Magic with slot 0 configs,
    // target position of 200 rotations
    m_motmag.Slot = 0;
    superStructure.setControl(m_motmag.withPosition(ampPosition));

    
  }

  public void rotateToSpeaker() {
    m_motmag.Slot = 0;
    superStructure.setControl(m_motmag.withPosition(0));

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Motor Position", superStructure.getPosition().getValueAsDouble());
  }
}
