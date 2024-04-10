package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimeLight;

public class SwerveJoystickCmd extends Command {
    private SwerveSubsystem swerveSubsystem;
    private LimeLight limeLightFront;
    private LimeLight limeLightRear;
    private Supplier <Double> xSpdFunction, ySpdFunction, turningSpdFunction, leftTrigger, rightTrigger;
    private Supplier <Boolean> visionAdjustmentFunction;
    private double kLimelightHorizontal = 0.08;
    private double txFront, rx, ry, theta;
    private double kLimelightForward = 1.3;
    private double kLimelightTurning =  0.1;
    private double targetHeading = 0;
    // private double superFastModeConstant = 7.5;
    private double slewMultiple = 2.0;
    private SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(DriveConstants.kPhysicalMaxSpeedMetersPerSecond * slewMultiple);
    private SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(DriveConstants.kPhysicalMaxSpeedMetersPerSecond * slewMultiple);
    private SlewRateLimiter slewRateLimiterTheta = new SlewRateLimiter(DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 999);
    private double xSpeed, ySpeed, turningSpeed;
    private boolean isFastModeActive;
    private Supplier <Boolean> bButtonFunction, xButtonFunction;
    //Robot is tippy in Y direction so we are decreasing yspeed
    private double yScaleFactor = (DriveConstants.ykTranslateDriveMaxSpeedMetersPerSecond/DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond);


    public  SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
                Supplier<Double> xSpdFunction, 
                Supplier<Double> ySpdFunction, 
                Supplier<Double> turningSpdFunction,
                Supplier<Boolean> visionAdjustmentFunction, 
                Supplier<Double> leftTrigger,
                Supplier<Double> rightTrigger,
                Supplier<Boolean> xButton,
                Supplier<Boolean> bButton,
                LimeLight limeLightFront,
                LimeLight limeLightRear){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.visionAdjustmentFunction = visionAdjustmentFunction;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.limeLightFront = limeLightFront;
        this.bButtonFunction = bButton;
        this.xButtonFunction = xButton;

        this.addRequirements(swerveSubsystem);


        // Register the sendable to LiveWindow and SmartDashboard
        SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
        SmartDashboard.putData(this);
    }
    public void setSlewMultiple(double value){
         this.slewMultiple = value;
            slewRateLimiterX = new SlewRateLimiter (DriveConstants.kPhysicalMaxSpeedMetersPerSecond * slewMultiple);
            slewRateLimiterY = new SlewRateLimiter (DriveConstants.kPhysicalMaxSpeedMetersPerSecond * slewMultiple);
    }

   

    @Override

    public void execute(){

        isFastModeActive = rightTrigger.get() > 0.2;

        boolean targetInView = limeLightFront.getVisionTargetStatus();
        boolean isAutoVisionActive = visionAdjustmentFunction.get();

        // Read in the robot xSpeed from controller
        xSpeed = processRawDriveSignal(xSpdFunction.get());
        xSpeed = applySpeedScaleToDrive(xSpeed);
        xSpeed = applySlewRateLimiter(xSpeed, slewRateLimiterX);
        
        // Read in the robot ySpeed from controller
        ySpeed = processRawDriveSignal(ySpdFunction.get());
        ySpeed = applySpeedScaleToDrive(ySpeed);
        ySpeed = applySlewRateLimiter(ySpeed, slewRateLimiterY);
    

        // Read in robot turningSpeed from controller
        turningSpeed = processRawDriveSignal(turningSpdFunction.get());
        turningSpeed = applySpeedScaleToTurn(turningSpeed);
        turningSpeed = applySlewRateLimiter(turningSpeed, slewRateLimiterTheta);

        // Apply speed reduction if commanded
        double superSlowMo = (1 - leftTrigger.get());
        superSlowMo = Math.max(0.3, superSlowMo);
        xSpeed *= superSlowMo;
        ySpeed *= superSlowMo;
        turningSpeed *= superSlowMo;

        if(targetInView && isAutoVisionActive){
            // fieldOrientedFunction = () -> false;
            // xSpeed = limeLightGrid.getVisionTargetAreaError() * kLimelightForward;
            // ySpeed = -limeLightGrid.getVisionTargetHorizontalError() * kLimelightHorizontal;
            //theta = swerveSubsystem.getHeading() - Math.atan2(xSpdFunction.get(), ySpdFunction.get())*(180.0/Math.PI);

            txFront = limeLightFront.getVisionTargetHorizontalError();
            rx = (ySpeed*Math.sin(swerveSubsystem.getHeading()*(Math.PI/180.0))+xSpeed*(Math.cos(swerveSubsystem.getHeading()*(Math.PI/180.0))));
            // ry = -1*txFront * kLimelightHorizontal; //translational
            turningSpeed = -1*txFront*kLimelightTurning;//rotational

        }

 
        
        
        // convert speeds to reference frames
        ChassisSpeeds chassisSpeeds;
        if(isAutoVisionActive){
            chassisSpeeds = new ChassisSpeeds(rx, ry, 0.0);
        }else {
            //need to define in constants.java
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        }
            
        chassisSpeeds.vyMetersPerSecond *= yScaleFactor;

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
        // for(int i = 0; i< moduleStates.length; i++){

        //     SmartDashboard.putNumber(String.format("module %d", i), moduleStates[i].speedMetersPerSecond);

        // }
        
        if(xButtonFunction.get() && slewMultiple != 2.0) {
            setSlewMultiple(2);
        }
        if(bButtonFunction.get() && slewMultiple != 6.0) {
            setSlewMultiple(6);
        }
    }

    /**
     * Process the controller input into the drive command by applying deadband and squaring
     * @param rawDriveSignal
     * @return
     */
    private double processRawDriveSignal(double rawDriveSignal){
        // Read in the robot xSpeed from controller
        boolean isOutsideDeadband = Math.abs(rawDriveSignal) > OIConstants.K_DEADBAND;
        double speed = isOutsideDeadband ? rawDriveSignal : 0.0;
        return Math.pow(speed,2) * Math.signum(speed);
    }

    private double applySpeedScaleToDrive(double processedDriveSignal){
        // If right trigger is pulled, used max speed, otherwise use translate speed
        double speedMultiplier = isFastModeActive ? DriveConstants.kPhysicalMaxSpeedMetersPerSecond : DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
        return processedDriveSignal *  speedMultiplier;
    }

     private double applySpeedScaleToTurn(double processedDriveSignal){
        // If right trigger is pulled, used max speed, otherwise use translate speed
        double speedMultiplier = isFastModeActive ? DriveConstants.kPhysicalMaxSpeedMetersPerSecond : DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.7;
        return processedDriveSignal *  speedMultiplier;
    }

    private double applySlewRateLimiter(double scaledDriveSpeed, SlewRateLimiter slewRateLimiter){
        // If right trigger is pulled, return slewRate figure, otherwise use scaled speed
        double slewedSpeed = slewRateLimiter.calculate(scaledDriveSpeed);
        // return isSlewActive ? slewedSpeed : scaledDriveSpeed;
        return slewedSpeed;
    }



    @Override
    public void end(boolean interupted){
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method super.initSendable(builder);
        // builder.addDoubleProperty("kLimeLightHorizontal", () -> kLimelightHorizontal, (value) -> kLimelightHorizontal = value);
        // builder.addDoubleProperty("kLimeLightForward", () -> kLimelightForward, (value) -> kLimelightForward = value);
        // builder.addBooleanProperty("targetInView", () -> limeLightGrid.getVisionTargetStatus(), null);
        // builder.addBooleanProperty("autoVisionFunction", () -> visionAdjustmentFunction.get(), null);
        // builder.addDoubleProperty("kLimeLightTurning", () -> kLimelightTurning, (value) -> kLimelightTurning = value);
        // builder.addDoubleProperty("targetHeading", () -> targetHeading, (value) -> targetHeading = value);
        builder.addDoubleProperty("x speed", () -> xSpeed, (value) -> xSpeed = value);
        builder.addDoubleProperty("Y speed", () -> ySpeed, (value) -> ySpeed = value);
        builder.addDoubleProperty("slewMultiple", () -> slewMultiple, (value) -> {
            this.slewMultiple = value;
            slewRateLimiterX = new SlewRateLimiter (DriveConstants.kPhysicalMaxSpeedMetersPerSecond * slewMultiple);
            slewRateLimiterY = new SlewRateLimiter (DriveConstants.kPhysicalMaxSpeedMetersPerSecond * slewMultiple);

          });
    
        // builder.addDoubleProperty("FPGA Clock", () -> Timer.getFPGATimestamp(), null);
        // builder.addDoubleProperty("rightTrigger.get()",() -> rightTrigger.get(), null);

    }

}
