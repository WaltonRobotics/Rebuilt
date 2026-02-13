package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.ShooterK.kLogTab;
import static frc.robot.Constants.ShooterK.kRobotToTurret;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.FuelSim;
import frc.robot.subsystems.Shooter;
import frc.util.WaltLogger;
import frc.util.WaltLogger.Pose3dLogger;
import frc.util.WaltLogger.Translation3dArrayLogger;

public class TurretVisualizer {
    private Translation3d[] trajectory = new Translation3d[50];
    private Supplier<Pose3d> poseSupplier;
    private Supplier<ChassisSpeeds> fieldSpeedSupplier;
    private final int CAPACITY = 30;

    private final Translation3dArrayLogger log_trajectory = WaltLogger.logTranslation3dArray(kLogTab, "trajectory");
    private final Pose3dLogger log_turretPose = WaltLogger.logPose3d(kLogTab, "turretPose");
    private final Pose3dLogger log_hoodAngle = WaltLogger.logPose3d(kLogTab, "hoodAngle");


    public TurretVisualizer(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedSupplier) {
        this.poseSupplier = poseSupplier;
        this.fieldSpeedSupplier = fieldSpeedSupplier;
    }

     private Translation3d launchVel(LinearVelocity vel, Angle angle) {
        Pose3d robot = poseSupplier.get();
        ChassisSpeeds fieldSpeeds = fieldSpeedSupplier.get();

        double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
        double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
        double xVel =
                horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
        double yVel =
                horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }

    private int fuelStored = 8;

    public boolean canIntake() {
        return fuelStored < CAPACITY;
    }

    public void intakeFuel() {
        fuelStored++;
    }

    public void launchFuel(LinearVelocity vel, Angle angle) {
        Pose3d robot = poseSupplier.get();

        Translation3d initialPosition = robot.getTranslation();
        FuelSim.getInstance().spawnFuel(initialPosition, launchVel(vel, angle));
    }

    public Command repeatedlyLaunchFuel(
            Supplier<LinearVelocity> velSupplier, Supplier<Angle> angleSupplier, Shooter shooter) {
        return shooter.runOnce(() -> launchFuel(velSupplier.get(), angleSupplier.get()))
                .andThen(Commands.waitSeconds(0.25))
                .repeatedly();
    }

     public void updateFuel(LinearVelocity vel, Angle angle) {
        Translation3d trajVel = launchVel(vel, angle);
        for (int i = 0; i < trajectory.length; i++) {
            double t = i * 0.04;
            double x = trajVel.getX() * t + poseSupplier.get().getTranslation().getX();
            double y = trajVel.getY() * t + poseSupplier.get().getTranslation().getY();
            double z = trajVel.getZ() * t
                    - 0.5 * 9.81 * t * t
                    + poseSupplier.get().getTranslation().getZ();

            trajectory[i] = new Translation3d(x, y, z);
        }
        log_trajectory.accept(trajectory);
    }

    public void update3dPose(Angle azimuthAngle, Angle hoodAngle) {
        log_turretPose.accept(new Pose3d(0,0,0, new Rotation3d(0,0,azimuthAngle.in(Radians))));
        Pose3d hoodPose = new Pose3d(0.1, 0, 0, new Rotation3d(0, hoodAngle.in(Radians), 0));
        hoodPose = hoodPose.rotateAround(new Translation3d(), new Rotation3d(0, 0, azimuthAngle.in(Radians)));
        hoodPose = new Pose3d(
            hoodPose.getTranslation().plus(kRobotToTurret.getTranslation()), hoodPose.getRotation());
        log_hoodAngle.accept(hoodPose);
    }
}
