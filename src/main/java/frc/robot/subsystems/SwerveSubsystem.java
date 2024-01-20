// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.enums.WheelPosition;

/** Add your docs here. */
public class SwerveSubsystem extends SubsystemBase{
    
    private ArrayList<SwerveModuleState> moduleStates = new ArrayList<>();
    private ArrayList<SwerveModule> swerveModules = new ArrayList<>();
    private double balanceConstant = (.007743);
    private double feedForwardConstant = (0);
    // private double previousRoll = 0;
    private double previousPitch = 0;
    // private double rollROC;
    private double pitchROC;
    // private double rollROCConstant = 0;
    private double pitchROCConstant = -2.027;
    private double errorMultiplier;
    private double xSpeed;
    private double integratorSum;
    private double integratorConstant = 0.0000;
    private double targetPitch = 0;
    private double saturatedPitch = -10;



    private final SwerveModule frontLeft = new SwerveModule(
        WheelPosition.FRONT_LEFT,
        Constants.RoboRioPortConfig.FRONT_LEFT_DRIVE,
        Constants.RoboRioPortConfig.FRONT_LEFT_TURN,
        false,
        false,
        Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_FRONT_LEFT,
        Constants.RoboRioPortConfig.FRONT_LEFT_CANCODER,
        Constants.RoboRioPortConfig.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        true,
        IdleMode.kBrake,
        IdleMode.kCoast
       );

    private final SwerveModule frontRight = new SwerveModule(
        WheelPosition.FRONT_RIGHT,
        Constants.RoboRioPortConfig.FRONT_RIGHT_DRIVE,
        Constants.RoboRioPortConfig.FRONT_RIGHT_TURN,
        false,
        false,
        Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_FRONT_RIGHT,
        Constants.RoboRioPortConfig.FRONT_RIGHT_CANCODER,
        Constants.RoboRioPortConfig.kFrontRightDriveAbsoluteEncoderOffsetRad,
        true,
        IdleMode.kBrake,
        IdleMode.kCoast
        );

    private final SwerveModule backLeft = new SwerveModule(
        WheelPosition.BACK_LEFT,
        Constants.RoboRioPortConfig.BACK_LEFT_DRIVE,
        Constants.RoboRioPortConfig.BACK_LEFT_TURN,
        false,
        false,
        Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_BACK_LEFT,
        Constants.RoboRioPortConfig.BACK_LEFT_CANCODER,
        Constants.RoboRioPortConfig.kBackLeftDriveAbsoluteEncoderOffsetRad,
        true,
        IdleMode.kBrake,
        IdleMode.kCoast
    );

    private final SwerveModule backRight = new SwerveModule(
        WheelPosition.BACK_RIGHT,
        Constants.RoboRioPortConfig.BACK_RIGHT_DRIVE,
        Constants.RoboRioPortConfig.BACK_RIGHT_TURN,
        false,
        false,
        Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_BACK_RIGHT,
        Constants.RoboRioPortConfig.BACK_RIGHT_CANCODER,
        Constants.RoboRioPortConfig.kBackRightDriveAbsoluteEncoderOffsetRad,
        true,
        IdleMode.kBrake,
        IdleMode.kCoast
        );

      private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition() };


    //idk if this is the gyro we have 
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final Pigeon2 pidgey = new Pigeon2(Constants.RoboRioPortConfig.PIGEON2);
    private StatusSignal<Double> yaw;
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        new Rotation2d(0),
        getModulePositions(),
        new Pose2d(0, 0, new Rotation2d(0)));

 
    private double kPXController =  AutoConstants.kPXController;
    private double kPYController = AutoConstants.kPYController;;
    private double kPThetaController = AutoConstants.kPThetaController;
    PIDController xController;
    PIDController yController;
    ProfiledPIDController thetaController;

    public SwerveSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                targetPitch = getPitchDegrees();
            } catch (Exception e) {
            }
        }).start(); 
        var toApply = new Pigeon2Configuration();
        pidgey.getConfigurator().apply(toApply);
        pidgey.setYaw(0, 0.1); // Set our yaw to 144 degrees and wait up to 100 ms for the setter to take affect
        pidgey.getYaw().waitForUpdate(0.1); // And wait up to 100 ms for the position to take affect
        

        /* Speed up signals to an appropriate rate */
        pidgey.getYaw().setUpdateFrequency(100);
        pidgey.getGravityVectorZ().setUpdateFrequency(100);
        // Initialize the moduleStates array
        moduleStates.add(new SwerveModuleState());
        moduleStates.add(new SwerveModuleState());
        moduleStates.add(new SwerveModuleState());
        moduleStates.add(new SwerveModuleState());

        //Initialize the x PID controller for autonomous swerve Controller command
        //xController = new PIDController(kPXController, 0, 0);
        
        // Initialize the swerveModules array
        this.swerveModules.add(this.frontLeft);
        this.swerveModules.add(this.frontRight);
        this.swerveModules.add(this.backLeft);
        this.swerveModules.add(this.backRight);

        // init for Path Planner
        this.initPathPlanner();

        //Register the sendables
        SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
        SmartDashboard.putData(this);

    }
//todo make sure we only do hardware call once (getAngle)

public double getRollDegrees(){
    return gyro.getRoll();
}

public double getPitchDegrees(){
    return gyro.getPitch();
}
public double getHeading(){
    //return Math.IEEEremainder(-(gyro.getAngle()), 360);
    return Math.IEEEremainder(-(yaw.getValueAsDouble()), 360);
}


public void zeroTurningEncoders(){
    for(int x=0; x<4; x++){
        swerveModules.get(x).zeroTurningEncoder();
    }
}
public Rotation2d getRotation2d(){

    return Rotation2d.fromDegrees(getHeading());
}

public Pose2d getPose(){
    return odometry.getPoseMeters();
}

public double getSaturatedPitch(){
    return saturatedPitch;
}
public void setSaturatedPitch(double x){
    saturatedPitch = x;
}
public PIDController getxController(){
    // DataLogManager.log(String.format("X conroller %.2f", kPXController));
    return new PIDController(kPXController, 0, 0);
}

public PIDController getyController(){
    // DataLogManager.log(String.format("Y controller %.2f", kPYController));
    return new PIDController(kPYController, 0,0);
}

public ProfiledPIDController getThetaController(){
    // DataLogManager.log(String.format("Theta controller %.2f", kThetaController));
    return new ProfiledPIDController(kPThetaController, 0, 0,AutoConstants.kThetaControllerConstraints);
}

public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition() };
}

public void resetOdometry(Pose2d pose){
    odometry.resetPosition( getRotation2d(), getModulePositions(), pose);
}

public void resetEncoders(){
    frontLeft.resetDriveEncoders();
    backLeft.resetDriveEncoders();
    frontRight.resetDriveEncoders();
    backRight.resetDriveEncoders();
}

public void zeroHeading(){
    gyro.reset();
}

public void zeroIntegrator(){
    integratorSum = 0;
}

/**
 * Sets the swerve module states in order (FL, FR, BL, BR)
 * @param desiredStates
 */
public void setModuleStates(SwerveModuleState[] desiredStates){
    moduleStates.clear();
    for(int i = 0; i < 4; i++){
        moduleStates.add(desiredStates[i]);
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
}

public ArrayList<SwerveModule> getSwerveModules(){
    return this.swerveModules;
}

public double autoBalance(){
    //rollROC = ((getRollDegrees() - previousRoll)/20);
    double currentPitch = getPitchDegrees();

    pitchROC = ((currentPitch - previousPitch)/ 20);

    //double balanceError = 0 - getRollDegrees();
    double balanceError = targetPitch - currentPitch;



    if(balanceError > -7.5 && balanceError < 7.5){
        integratorSum += balanceError * 20;
    }

    if (balanceError < 0) {
        errorMultiplier = -1;
    } else {
        errorMultiplier = 1;
    }
    double sqrBalanceError = (Math.pow(balanceError, 2)) * errorMultiplier;
    
    //rollROC (rate of change) is in Degrees/Milisecond
    double proportionalSpeed = (balanceConstant * balanceError) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
    //deriv speed -4.16 proportional speed .0033
    //double derivSpeed = ((rollROC * rollROCConstant) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond);
    double derivSpeed = ((pitchROC * pitchROCConstant) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond);
    double feedForwardSpeed = ((feedForwardConstant * sqrBalanceError) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond);
    double integratorSpeed = ((integratorConstant * integratorSum) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond);
    //derivSpeed = Math.min(Math.abs(proportionalSpeed + feedForwardSpeed), Math.abs(derivSpeed)) * Math.signum(derivSpeed);
    // SmartDashboard.putNumber("proportional speed", proportionalSpeed);
    // SmartDashboard.putNumber("deriv Speed", derivSpeed);
    // SmartDashboard.putNumber("feed forward speed", feedForwardSpeed);
    // SmartDashboard.putNumber("integrator speed", integratorSpeed);


    xSpeed = proportionalSpeed + derivSpeed + feedForwardSpeed + integratorSpeed;
    //previousRoll = getRollDegrees();
    previousPitch = currentPitch;

    return xSpeed;
}

public void stopModules(){
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    this.setModuleStates(moduleStates);
}


/**
 * Configures Path Planner objects
 * --AutoBuilder
 * --Managed Named Commands
 */
private void initPathPlanner(){
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotRelativeSpeeds,
      this::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        new PIDConstants(AutoConstants.kPXController,0,0),
        new PIDConstants(AutoConstants.kPThetaController,0,0),
        AutoConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kSpinRadius,
        new ReplanningConfig()
      ),
      () -> false,
      this
    );
}

/**
 * Converts chassis speeds (robot relative speed) to swerve module states and then applies those states
 * @param chassisSpeeds
 */
public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    // Convert the chassis speeds to module states
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    // apply those states to the swerve modles
    this.setModuleStates(swerveModuleStates);
}

/**
 * Calculates the chassis speed from the from the module states (used for odometry)
 * @return
 */
public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                                                           frontRight.getState(),
                                                           backLeft.getState(),
                                                           backRight.getState());
} 


@Override
public void periodic() {
    odometry.update(getRotation2d(), getModulePositions());
     yaw = pidgey.getYaw();
   

   // System.out.println("Yaw is " + yaw.getValueAsDouble() + " with " + yaw.getTimestamp().getLatency());

    SmartDashboard.putNumber("PigeonYaw",yaw.getValueAsDouble());
    SmartDashboard.putNumber("PigeonYawRemainder",Math.IEEEremainder(-(yaw.getValueAsDouble()), 360));
    // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    // SmartDashboard.putNumber("Robot Location x:", getPose().getX());
    // SmartDashboard.putNumber("Robot Location Y", getPose().getY());
    // System.out.println("Robot Locaton X:" + getPose().getX());
    // System.out.println("Robot Location Y:" + getPose().getY());
    // for(int i = 0; i < modulePositions.length; i++ ){
    //     System.out.println(modulePositions[i]);
    // }
    
    // SmartDashboard.putNumber("Loop Count: ", loopCount++);
    // DataLogManager.log(String.format("Loop count %d", loopCount));

    for (SwerveModule swerveModule: swerveModules){
        // SmartDashboard.putNumber(String.format("%s Angle", swerveModule.wheelPosition.name()), swerveModule.getAbsoluteEncoderRadians());
        SmartDashboard.putNumber(String.format("%s Angle", swerveModule.wheelPosition.name()), swerveModule.getAbsoluteEncoderRadians());
        SmartDashboard.putNumber(String.format("%s Position", swerveModule.wheelPosition.name()), swerveModule.getDrivePosition());
      //  SmartDashboard.putNumber(String.format("%s Back Left", swerveMo))
        SmartDashboard.putNumber(String.format("%s Turning Encoder", swerveModule.wheelPosition.name()), swerveModule.getTurningPosition());
        // SmartDashboard.putNumber(String.format("%s Target Angle", swerveModule.wheelPosition.name()), swerveModule.getState().angle.getRadians());
        SmartDashboard.putNumber(String.format("%s Velocity", swerveModule.wheelPosition.name()), swerveModule.getDriveVelocity());
    }
    // SmartDashboard.putNumber("FL Angle", frontLeft.getAbsoluteEncoderRadians());
    // SmartDashboard.putNumber("FL Turning Encoder", frontLeft.getTurningPosition());
    // SmartDashboard.putNumber("FR Angle", frontRight.getAbsoluteEncoderRadians());
    // SmartDashboard.putNumber("BL Angle", backLeft.getAbsoluteEncoderRadians());
    // SmartDashboard.putNumber("BR Angle", backRight.getAbsoluteEncoderRadians());
    // SmartDashboard.putNumber("BL Encoder Voltage", backLeft.getAbsoluteEncoder().getVoltage());
    // SmartDashboard.putNumber("5V RobotController", RobotController.getCurrent5V());
    
    // SmartDashboard.putNumber("FL Target Angle", moduleStates.get(0).angle.getRadians());
    // SmartDashboard.putNumber("Gyro", gyro.getAngle());
   // SmartDashboard.putNumber("Mystery", getHeading());
       SmartDashboard.putNumber ("Pitch", gyro.getPitch());
//    SmartDashboard.putNumber ("Roll", gyro.getRoll());
   
    // DataLogManager.log(String.format("Back Left Encoder Voltage %f", backLeft.getAbsoluteEncoder().getVoltage()));
    // DataLogManager.log(String.format("Back Right Encoder Voltage %f", backRight.getAbsoluteEncoder().getVoltage()));


    // DataLogManager.log(String.format("Back left Angle %f", backLeft.getAbsoluteEncoderRadians()));
    // DataLogManager.log(String.format("Back right Angle %f", backRight.getAbsoluteEncoderRadians()));
    // DataLogManager.log(String.format("Front left Angle %f", frontLeft.getAbsoluteEncoderRadians()));
    // DataLogManager.log(String.format("Front right Angle %f", frontRight.getAbsoluteEncoderRadians()));

    // DataLogManager.log(String.format("Voltage %f", RobotController.getCurrent5V()));

    // SmartDashboard.putNumber("Integrator Sum", integratorSum);
}

@Override
public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    // builder.addDoubleProperty("FL Power", () -> frontLeft.getDriveVelocity(), null);
    // builder.addDoubleProperty("FR Power", () -> frontRight.getDriveVelocity(), null);
    // builder.addDoubleProperty("BL Power", () -> backLeft.getDriveVelocity(), null);
    // builder.addDoubleProperty("BR Power", () -> backRight.getDriveVelocity(), null);
     builder.addDoubleProperty("kPXController", () -> kPXController, (value) -> kPXController = value);
     builder.addDoubleProperty("kPYController", () -> kPYController, (value) -> kPYController = value);
     builder.addDoubleProperty("kThetaController", () -> kPThetaController, (value) -> kPThetaController = value);

    builder.addDoubleProperty("balanceConstant", () -> balanceConstant, (value) -> balanceConstant = value);
    // builder.addDoubleProperty("Roll Rate of Change", () -> rollROC, null);
//    builder.addDoubleProperty("Roll Rate of Change Constant", () -> rollROCConstant, (value) -> rollROCConstant = value);
   builder.addDoubleProperty("Roll Rate of Change Constant", () -> pitchROCConstant, (value) -> pitchROCConstant = value);

    builder.addDoubleProperty("feed forward", () -> feedForwardConstant, (value) -> feedForwardConstant = value);
    builder.addStringProperty("Odometry Position", () -> this.odometry.getPoseMeters().toString(), null);

    // builder.addDoubleProperty("Integrator Constant", () -> integratorConstant, (value) -> integratorConstant = value);



}

}


