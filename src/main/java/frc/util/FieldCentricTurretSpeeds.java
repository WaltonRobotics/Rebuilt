package frc.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShooterK;

/**
 * A class with utility methods for converting robot- or field-centric ChassisSpeeds into a ChassisSpeeds object representing the Turret Speeds.
 * <p> Note that omega is irrelevant and set to 0 for the Turret Speeds as the rotation of the tunnel no longer affects how balls enter the shooter.
 */
public class FieldCentricTurretSpeeds {
    public static ChassisSpeeds fromRobotRelativeSpeeds(ChassisSpeeds robotChassisSpeeds, Rotation2d robotAngle) {
        ChassisSpeeds fieldCentricChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotChassisSpeeds, robotAngle);

        return fromFieldRelativeSpeeds(fieldCentricChassisSpeeds, robotAngle);
    }

    public static ChassisSpeeds fromFieldRelativeSpeeds(ChassisSpeeds fieldCentricChassisSpeeds, Rotation2d robotAngle) {
        double vxMetersPerSecond = fieldCentricChassisSpeeds.vxMetersPerSecond,
               vyMetersPerSecond = fieldCentricChassisSpeeds.vyMetersPerSecond,
               omegaRadiansPerSecond = fieldCentricChassisSpeeds.omegaRadiansPerSecond;
        double vTangentialMetersPerSecond = ShooterK.kTurretOffset_m * omegaRadiansPerSecond;
        double turretVxMetersPerSecond = vTangentialMetersPerSecond * Math.sin(Degrees.of(ShooterK.kTurretAngleFromCenterDeg + robotAngle.getDegrees()).in(Radians));
        double turretVyMetersPerSecond = vTangentialMetersPerSecond * Math.cos(Degrees.of(ShooterK.kTurretAngleFromCenterDeg + robotAngle.getDegrees()).in(Radians));
        
        ChassisSpeeds fieldCentricTurretSpeeds = new ChassisSpeeds(vxMetersPerSecond + turretVxMetersPerSecond, vyMetersPerSecond + turretVyMetersPerSecond, 0);
        return fieldCentricTurretSpeeds;
    }
}