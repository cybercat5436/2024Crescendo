package frc.robot.enums;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Midfield Note positions, starting with A => Amp side
 * Based on this field drawing: https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
 */
public enum MidfieldNote {
    
    // 
    A(new Translation2d(8.293, Units.inchesToMeters(29.74 + (4 * 66)))),
    B(new Translation2d(8.293, Units.inchesToMeters(29.74 + (3 * 66)))),
    C(new Translation2d(8.293, Units.inchesToMeters(29.74 + (2 * 66)))),
    D(new Translation2d(8.293, Units.inchesToMeters(29.74 + (1 * 66)))),
    E(new Translation2d(8.293, Units.inchesToMeters(29.74 + (0 * 66))));


    private Translation2d translation2d;
    

    MidfieldNote(Translation2d translation2d){
        this.translation2d = translation2d;
    }

    public Translation2d getTranslation2d(){
        return this.translation2d;
    }
    
}
