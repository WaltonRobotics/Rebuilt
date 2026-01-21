package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;



public class ShooterTable {
    private static double minDistance;
    private static double maxDistance;
    private static InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = 
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static InterpolatingDoubleTreeMap shotFlywheelSpeedMap = 
        new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap timeOfFlightMap = 
        new InterpolatingDoubleTreeMap();

    
    //lookup table produced from MA 1/20/26
    static {
        minDistance = 1.17;
        maxDistance = 5.63;
    
        shotHoodAngleMap.put(1.17, Rotation2d.fromDegrees(19.0));
        shotHoodAngleMap.put(1.55, Rotation2d.fromDegrees(19.0));
        shotHoodAngleMap.put(1.96, Rotation2d.fromDegrees(19.0));
        shotHoodAngleMap.put(2.14, Rotation2d.fromDegrees(20.0));
        shotHoodAngleMap.put(2.40, Rotation2d.fromDegrees(21.0));
        shotHoodAngleMap.put(2.78, Rotation2d.fromDegrees(22.0));
        shotHoodAngleMap.put(3.10, Rotation2d.fromDegrees(24.0));
        shotHoodAngleMap.put(3.44, Rotation2d.fromDegrees(26.0));
        shotHoodAngleMap.put(3.84, Rotation2d.fromDegrees(27.0));
        shotHoodAngleMap.put(4.60, Rotation2d.fromDegrees(32.0));
        shotHoodAngleMap.put(5.12, Rotation2d.fromDegrees(33.0));
        shotHoodAngleMap.put(5.63, Rotation2d.fromDegrees(36.0));
    
        shotFlywheelSpeedMap.put(1.17, 185.0);
        shotFlywheelSpeedMap.put(1.55, 205.0);
        shotFlywheelSpeedMap.put(1.96, 225.0);
        shotFlywheelSpeedMap.put(2.14, 225.0);
        shotFlywheelSpeedMap.put(2.40, 225.0);
        shotFlywheelSpeedMap.put(2.78, 230.0);
        shotFlywheelSpeedMap.put(3.10, 235.0);
        shotFlywheelSpeedMap.put(3.44, 238.0);
        shotFlywheelSpeedMap.put(3.84, 245.0);
        shotFlywheelSpeedMap.put(4.60, 252.0);
        shotFlywheelSpeedMap.put(5.12, 265.0);
        shotFlywheelSpeedMap.put(5.63, 270.0);
    
        timeOfFlightMap.put(5.68, 1.16);
        timeOfFlightMap.put(4.55, 1.12);
        timeOfFlightMap.put(3.15, 1.11);
        timeOfFlightMap.put(1.88, 1.09);
        timeOfFlightMap.put(1.38, 0.90);
    }
    public static getSpeed(double normDistance) {
        
    } 
}
