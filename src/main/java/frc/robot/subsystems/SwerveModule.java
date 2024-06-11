// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.enums.WheelPosition;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class SwerveModule implements Sendable{

  private final CANSparkFlex driveMotor;
  private final CANSparkFlex turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final PIDController turningPidController;
  private final SparkPIDController velocityPidController;

  // private final AnalogInput absoluteEncoder;
  private final CANcoder cancoder;
  private final CANcoderConfigurator cancoderConfigurator;
  private final double absoluteEncoderOffsetRotations;
  private double driveMotorPower;

  public final WheelPosition wheelPosition;

  public double kP = 0.45;
  public double kFF = 0.225;
  
  public SwerveModuleState desiredState = new SwerveModuleState();

  /** Creates a new SwerveModule. */
  public SwerveModule(WheelPosition wheelPosition, int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
   int cancoderId, double absoluteEncoderOffsetRotations, IdleMode driveMode, IdleMode turningMode) {

    this.wheelPosition = wheelPosition;
    this.absoluteEncoderOffsetRotations = absoluteEncoderOffsetRotations;
    // absoluteEncoder = new AnalogInput(absoluteEncoderId);
    cancoder = new CANcoder(cancoderId, Constants.RoboRioPortConfig.Canivore);
    driveMotor = new CANSparkFlex(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkFlex(turningMotorId, MotorType.kBrushless);

    cancoderConfigurator = cancoder.getConfigurator();
    cancoderConfigurator.apply(new MagnetSensorConfigs().withMagnetOffset(absoluteEncoderOffsetRotations));

    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);



    driveMotor.clearFaults();
    turningMotor.clearFaults();

    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();

    driveMotor.setSmartCurrentLimit(100, 100);
    turningMotor.setSmartCurrentLimit(40, 40);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveMotor.setIdleMode(driveMode);
    turningMotor.setIdleMode(turningMode);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    velocityPidController = driveMotor.getPIDController();
    velocityPidController.setP(kP);
    velocityPidController.setFF(kFF);

    resetDriveEncoders();
    resetTurningEncoderWithAbsolute();
    System.out.println("Setting " + wheelPosition + " with offset of " + absoluteEncoderOffsetRotations);

    //Register the sendables
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.wheelPosition.toString());
    SmartDashboard.putData(this);
  }

  /*
  public double getDrivePosition() {
    return driverEncoder.getPosition();
  }

  public double getSpinPosition() {
    return spinEncoder.getPosition();
  }
  
  public double getDriveVelocity() {
    return driverEncoder.getVelocity();
  }

  public double getSpinVelocity() {
    return spinEncoder.getVelocity();
  }

  public void resetEncoders() {
    driverEncoder.setPosition(0);
    spinEncoder.setPosition(0);
  }
  */

  public void resetDriveEncoders() {
    driveEncoder.setPosition(0.0);
    
    // DataLogManager.log(String.format("About to reset encoders for position %s", this.wheelPosition.name()));
    // DataLogManager.log(String.format("turning Encoder %.2f", turningEncoder.getPosition()));
    // DataLogManager.log(String.format("absoluteEncoder %.2f", this.getAbsoluteEncoderRadians()));
    // turningEncoder.setPosition(getAbsoluteEncoderRadians());
    // DataLogManager.log(String.format("After reset encoders for position %s", this.wheelPosition.name()));
    // DataLogManager.log(String.format("after zero turningEncoder %.2f", turningEncoder.getPosition()));
    // DataLogManager.log(String.format("after zero absoluteEncoder %.2f", this.getAbsoluteEncoderRadians()));
  }
  public void resetTurningEncoderWithAbsolute () {
    turningEncoder.setPosition(getAbsoluteEncoderRadians());
    // cancoder.setPosition(cancoder.getAbsolutePosition().getValueAsDouble());
  }
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition( driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
  }
  public void zeroTurningEncoder(){
    turningEncoder.setPosition(0);
    // cancoder.setPosition(0);  
  }

  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  public double getTurningPosition(){
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRadians(){
    // return this.absoluteEncoder;
    double rads = cancoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI;
    // if (absoluteEncoderReversed){
    //   rads *= -1;
    // } 
    //rads -= absoluteEncoderOffsetRad;
    rads = boundAngle(rads);
    return rads;
  }




  public SwerveModuleState getActualState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }
  public SwerveModuleState getDesiredState(){
    return new SwerveModuleState(this.desiredState.speedMetersPerSecond, this.desiredState.angle);
  }

  //definetly ot right but gets rid of error
  // private SwerveModuleState SwerveModuleState(double driveVelocity, Rotation2d rotation2d) {
  //   return null;
  // }

  /**
   * Applies the provided state to the individual module
   * @param state
   */
  public void setDesiredState(SwerveModuleState state){
    
    this.desiredState = state;

    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      this.desiredState = new SwerveModuleState(0.0, state.angle);
      this.stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getActualState().angle);

    // Setting Drive Motor Power Using a Linear Correlation with Max Speed
    // driveMotorPower = state.speedMetersPerSecond /DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    // driveMotor.set(driveMotorPower);

    // System.out.println("Turning Encoder Error: "+(state.angle.getRadians()-boundAngle(turningEncoder.getPosition())));

    velocityPidController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

    turningMotor.set(turningPidController.calculate(turningEncoder.getPosition(), state.angle.getRadians()));
  
    // SmartDashboard.putString(String.format("%s Modue State", this.wheelPosition.name()), state.toString());
    
    // SmartDashboard.putNumber(String.format("%s Drive Motor Power", this.wheelPosition.name()) , driveMotorPower);

  }


  // public double getAbsoluteEncoderRadians() {
  //   // double angle = absoluteEncoder.getVoltage() / RobotController.getCurrent5V();
  //   double angle = absoluteEncoder.getVoltage() / 5.0;
  //   angle = (Math.PI * 2) * angle - Math.PI;
  //   if (absoluteEncoderReversed){
  //     angle *= -1;
  //   } 
  //   angle -= absoluteEncoderOffsetRad;
  //   angle = boundAngle(angle);
  //   return angle;
  // }

  public double boundAngle(double inputAngleRad){
    // double outputAngleRad = inputAngleRad;
    // while(Math.abs(outputAngleRad)>=Math.PI){
    //   if (outputAngleRad <= -Math.PI){
    //   outputAngleRad += (2*Math.PI);
    // } else if(outputAngleRad > (Math.PI)){
    //   outputAngleRad -= (2 * Math.PI);
    // }
    // }
    // return outputAngleRad;
      return Math.IEEEremainder(inputAngleRad, 2*Math.PI);
  }



  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public double getDrivePower(){
    return driveMotorPower;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    builder.addDoubleProperty("Drive Velocity", () -> this.getDriveVelocity(), null);
    builder.addDoubleProperty("CANCoder Absolute", () -> cancoder.getAbsolutePosition().getValueAsDouble(),null);
    // builder.addDoubleProperty("Desired Drive Power", () -> this.getDrivePower()*DriveConstants.kPhysicalMaxSpeedMetersPerSecond, null);
    builder.addDoubleProperty("Desired Drive Power", () -> this.desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty("Turning Encoder",() -> this.getTurningPosition(),null);
    // builder.addDoubleProperty("Physical Restraints", () -> DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, null);
  
  }




  /**@Override
  public void periodic() {
   This method will be called once per scheduler run
  } **/
}
