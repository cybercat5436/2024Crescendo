// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Util {
    public Command getPath(Pose2d targetPose){
        // Create the constraints to use while pathfinding
     PathConstraints constraints = new PathConstraints(
      3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
      // Since AutoBuilder is configured, we can use it to build pathfinding commands
     return AutoBuilder.pathfindToPose(
     targetPose,constraints,0.0, // Goal end velocity in meters/sec
     0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
    }


}
