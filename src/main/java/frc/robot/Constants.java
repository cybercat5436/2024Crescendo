// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.12;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = .5;
    }

    public class RoboRioPortConfig{
        public static final int PDP = 0;
        public static final int PDH = 1;
        public static final String Canivore = "carter";

            
        

        public static final int FRONT_LEFT_DRIVE = 8;
        public static final int FRONT_RIGHT_DRIVE = 3;
        public static final int BACK_LEFT_DRIVE = 6;
        public static final int BACK_RIGHT_DRIVE = 4;
        public static final int FRONT_LEFT_TURN = 7;
        public static final int FRONT_RIGHT_TURN = 10;
        public static final int BACK_LEFT_TURN = 5;
        public static final int BACK_RIGHT_TURN = 2;
        public static final int PIGEON2 = 9;
        public static final int INTAKE_MOTOR = 11;
        public static final int FRONT_LEFT_CANCODER = 15;
        public static final int FRONT_RIGHT_CANCODER = 18;
        public static final int BACK_LEFT_CANCODER = 17;
        public static final int BACK_RIGHT_CANCODER = 16;
        public static final int ARM_MOTOR = 21;
        public static final int SPEAKER_LAUNCHER = 22;
        public static final int SPEAKER_FEEDER_MOTOR = 23;
        public static final int SPEAKER_FALCON_MOTOR = 24; 
        public static final int AMP_FALCON_MOTOR = 25;   
        public static final int SUPERSTRUCTURE_MOTOR = 28;    
        public static final int CLAW_MOTOR = 31;
        public static final int ORIENT_MOTOR = 42;
        public static final int EXTENDER_MOTOR = 51;
        public static final int CLIMBER_RIGHT = 62;
        public static final int CLIMBER_LEFT = 61;
    
        public static final int ABSOLUTE_ENCODER_FRONT_LEFT = 0;
        public static final int ABSOLUTE_ENCODER_FRONT_RIGHT = 1;
        public static final int ABSOLUTE_ENCODER_BACK_LEFT = 2;
        public static final int ABSOLUTE_ENCODER_BACK_RIGHT = 3;

        public static final int EXTENDER_ZERO_LIMIT_SWITCH = 0;
        public static final int EXTENDER_MAX_LIMIT_SWITCH = 1;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRotations = 0.347900 ;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRotations = -0.413574;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRotations = 0.164307 ;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRotations = -0.071045 ;
        
        
    }


    public static class DriveConstants{
        public static final double kTranslateDriveMaxSpeedMetersPerSecond = 1.8; //normal speed
        public static final double ykTranslateDriveMaxSpeedMetersPerSecond = 1.8;
        public static final double kRotateDriveMaxSpeedMetersPerSecond = 5.82;//4.36; //super fast mode
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.82;//4.36;
        //public static final double kTrackWidth = Units.inchesToMeters(19); //OLD
        public static final double kTrackWidth = 0.524;
        //kWheelBase: Distance between front and back of the robot
        //public static final double kWheelBase = Units.inchesToMeters(23.5);
        public static final double kWheelBase = 0.645;
        public static final double kSpinRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase,2)) / 2.0;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kRotateDriveMaxSpeedMetersPerSecond/kSpinRadius;//3.78;

        //public static final double kLimelightHorizontal = 0.0667;
        //public static final double kLimelightForward = 0.0833; //CHANGE
    }

    public static class OIConstants{
        public static final double K_DEADBAND = 0.10;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = 
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = kMaxAngularSpeedRadiansPerSecond * 2.0;  // half a second to accelerate to full speed
        //used to be 1.5
        public static final double kPXController = 2.9;
        public static final double kPYController = 2.9;

        //Test with 2.9 * 2, which was set in swervesubsystem auton
        public static final double kPThetaController = 5.0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

}
