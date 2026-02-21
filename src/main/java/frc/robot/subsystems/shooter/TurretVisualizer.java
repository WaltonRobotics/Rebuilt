// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.ShooterK.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.util.WaltLogger.Pose3dLogger;
import frc.util.WaltLogger.Translation3dArrayLogger;

import java.util.function.Supplier;

/** Add your docs here. */
public class TurretVisualizer {
    private Translation3d[] trajectory = new Translation3d[50];
    private Supplier<Pose3d> m_poseSupplier;
    private Supplier<ChassisSpeeds> m_fieldSpeedsSupplier;
    private final Translation3dArrayLogger log_trajectoryArray = new Translation3dArrayLogger(kLogTab, "fuelTrajectory");
    private final Pose3dLogger log_turretPose = new Pose3dLogger(kLogTab, "turretPose"); 
    private final Pose3dLogger log_hoodPose = new Pose3dLogger(kLogTab, "hoodPose");

    public Shooter m_shooter;

    public TurretVisualizer(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        m_poseSupplier = poseSupplier;
        m_fieldSpeedsSupplier = fieldSpeedsSupplier;
    }

    private Translation3d launchVel(LinearVelocity vel, Angle angle) {
        Pose3d robot = m_poseSupplier.get();
        ChassisSpeeds fieldSpeeds = m_fieldSpeedsSupplier.get();

        double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
        double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
        double xVel = horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
        double yVel = horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }

    public void updateFuel(LinearVelocity vel, Angle angle) {
        Translation3d trajVel = launchVel(vel, Degrees.of(90).minus(angle));
        for (int i = 0; i < trajectory.length; i++) {
            double t = i * 0.04;
            double x = trajVel.getX() * t + m_poseSupplier.get().getTranslation().getX();
            double y = trajVel.getY() * t + m_poseSupplier.get().getTranslation().getY();
            double z = trajVel.getZ() * t
                    - 0.5 * 9.81 * t * t
                    + m_poseSupplier.get().getTranslation().getZ();

            trajectory[i] = new Translation3d(x, y, z);
        }

        log_trajectoryArray.accept(trajectory);
    }

    public void update3dPose(Angle azimuthAngle, Angle hoodAngle) {
        Pose3d turretPose = new Pose3d( m_poseSupplier.get().getX(), m_poseSupplier.get().getY(), m_poseSupplier.get().getZ(), new Rotation3d(0, 0, azimuthAngle.in(Radians)));
        log_turretPose.accept(turretPose);
        Pose3d hoodPose = new Pose3d(0.1, 0, 0, new Rotation3d(0, hoodAngle.in(Radians), 0));
        hoodPose = hoodPose.rotateAround(new Translation3d(), new Rotation3d(0, 0, azimuthAngle.in(Radians)));
        hoodPose = new Pose3d(
                hoodPose.getTranslation().plus(kRobotToTurret.getTranslation()), hoodPose.getRotation());
        log_hoodPose.accept(hoodPose);
    }
}