
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShooterK.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.FieldConstants;
import frc.util.WaltLogger.*;
import edu.wpi.first.units.measure.Time;

public class ShotCalculator {
    private static final DoubleLogger log_timeOfFlight = new DoubleLogger("Shooter/Calculator", "timeOfFlight");
    private static final DoubleLogger log_distToTargetMeters = new DoubleLogger("Shooter/Calculator", "distTargetToMeters");

    private static final double kMetersToInches = 1.0 / 0.0254;
    // private static final Tracer m_iterativeTracer = new Tracer();

    //see 5000's code (circa 2/16/2026 9:11 PM EST)
    public static final InterpolatingTreeMap<Double, ShotData> m_shotMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShotData::interpolate);

    public static final InterpolatingDoubleTreeMap m_timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private static final double minDistance;
    private static final double maxDistance;

    private static final double kRPSBoost = 5.0;

    //LERP MADE ON 3/15/2025
    static {
        //TODO: find the actual minDistance and maxDistance for shooting
        minDistance = 0.94;
        maxDistance = 5.8;

        //Ordered via DistanceToTarget
        addLerpPoint(5.700, 59.47, 0.88, 1.33);
        addLerpPoint(5.628, 59.47, 0.91, 1.39);
        addLerpPoint(5.515, 59.50, 0.87, 1.30);
        addLerpPoint(5.412, 58.50, 0.86, 1.27);
        addLerpPoint(5.303, 58.47, 0.90, 1.65);
        addLerpPoint(5.060, 57.00, 0.85, 1.26);
        addLerpPoint(4.910, 56.00, 0.81, 1.17);
        addLerpPoint(4.874, 55.50, 0.80, 1.29);
        addLerpPoint(4.684, 54.00, 0.77, 1.21);
        addLerpPoint(4.562, 54.50, 0.55, 1.13);
        addLerpPoint(4.496, 55.86, 0.45, 1.40);
        addLerpPoint(4.407, 55.86, 0.45, 1.38);
        addLerpPoint(4.108, 55.86, 0.42, 1.40);
        addLerpPoint(4.000, 55.46, 0.42, 1.37);
        addLerpPoint(3.944, 54.90, 0.37, 1.33);
        addLerpPoint(3.823, 55.20, 0.20, 1.47);
    }

    public static void addLerpPoint(double distanceToTarget, double shooterRPS, double hoodRots, double timeOfFlight) {
        m_shotMap.put(distanceToTarget, new ShotData(RotationsPerSecond.of(shooterRPS), Rotations.of(hoodRots)));
        m_timeOfFlightMap.put(distanceToTarget, timeOfFlight);
    }

    
    /**
     * Gets the 2D distance from turret pivot to target in meters, using raw doubles.
     * Zero-allocation hot-path version.
     */
    public static double getDistanceToTargetM(double robotX, double robotY, double robotHeadingRad,
            double targetX, double targetY) {
        double cosR = Math.cos(robotHeadingRad);
        double sinR = Math.sin(robotHeadingRad);
        double turretX = robotX + kTurretOffsetX_m * cosR - kTurretOffsetY_m * sinR;
        double turretY = robotY + kTurretOffsetX_m * sinR + kTurretOffsetY_m * cosR;
        double dx = turretX - targetX;
        double dy = turretY - targetY;
        double dist = Math.sqrt(dx * dx + dy * dy);
        log_distToTargetMeters.accept(dist);
        return dist;
    }

    /**
     * Gets the Distance from current robot position to desired target.
     * Allocates Pose3d/Distance — use {@link #getDistanceToTargetM} on hot paths.
     */
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        Pose3d turretPose = new Pose3d(robot).transformBy(kTurretTransform);
        double dist = getDistanceToTargetM(
            turretPose.getX(), turretPose.getY(), turretPose.getRotation().getAngle(),
            target.getX(), target.getY());
        log_distToTargetMeters.accept(dist);
        return Meters.of(dist);
    }

    // see https://www.desmos.com/geometry/l4edywkmha
    public static Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity,
            Translation3d target) {
        double vel = velocity.in(InchesPerSecond);
        double x_dist = getDistanceToTargetM(
            robot.getX(), robot.getY(), robot.getRotation().getRadians(),
            target.getX(), target.getY()) * kMetersToInches;
        double y_dist = target.getZ() * kMetersToInches - kTurretOffsetZ_in;

        double angle = Math.atan(((vel * vel) + Math.sqrt(
                Math.pow(vel, 4) - kGravity * (kGravity * x_dist * x_dist + 2 * y_dist * vel * vel)))
                / (kGravity  * x_dist));

        return Radians.of(angle);
    }

    //calculate how long it will take for a projectile to travel a certain amount of distance given its initial velocity and angle
    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle,
            Distance distance) {
        double tofSec = calculateTimeOfFlightSec(
            exitVelocity.in(MetersPerSecond), hoodAngle.in(Radians), distance.in(Meters));
        return Seconds.of(tofSec);
    }

    /** Raw-double TOF in seconds. Zero-allocation. */
    public static double calculateTimeOfFlightSec(double velMps, double hoodAngleRad, double distM) {
        double launchAngle = Math.PI / 2 - hoodAngleRad;
        double tofSec = distM / (velMps * Math.cos(launchAngle));
        log_timeOfFlight.accept(tofSec);
        return tofSec;
    }

    public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
    }

    /** Raw double: rad/s from m/s and radius in meters. */
    public static double linearToAngularVelocityRadPerSec(double mps, double radiusM) {
        return mps / radiusM;
    }

    /** Raw double: m/s from rad/s and radius in meters. */
    public static double angularToLinearVelocityMps(double radPerSec, double radiusM) {
        return radPerSec * radiusM;
    }

    public static double getMinTimeOfFlight() {
        return m_timeOfFlightMap.get(minDistance);
    }

    public static double getMaxTimeOfFlight() {
        return m_timeOfFlightMap.get(maxDistance);
    }

    /**
     * Move a target a set time in the future along a velocity defined by
     * fieldSpeeds
     * Integral for SOTM, as this is what accounts for the speed the Robot is
     * going.
     * 
     * @param target desired target
     * @param fieldSpeeds curret robotVelocity
     * @param timeOfFlight timeOfFlight from calculations or LERP table
     * @return where we will need to shoot to account for us moving.
     */
    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds,
            Time timeOfFlight) {
        double predictedX = target.getX()
                - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds); //need time of flight b/c that tells you how close/far you can shoot to the target according to speeds.
        double predictedY = target.getY()
                - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Translation3d(predictedX, predictedY, target.getZ());
    }

    // https://www.desmos.com/calculator/ezjqolho6g
    // If you're having trouble understanding this method, go mess with the values in desmos / read what those do, as this is just the same desmos calc.
    public static ShotData calculateShotFromFunnelClearance(Pose2d robot,
            Translation3d actualTarget, Translation3d predictedTarget) {
        return calculateShotFromFunnelClearance(robot,
                actualTarget.getX(), actualTarget.getY(),
                predictedTarget.getX(), predictedTarget.getY(), predictedTarget.getZ());
    }

    /** Raw-double overload — zero allocation in the hot loop. */
    static ShotData calculateShotFromFunnelClearance(Pose2d robot,
            double actualTargetX, double actualTargetY,
            double predX, double predY, double predZ) {
        double robotX = robot.getX();
        double robotY = robot.getY();
        double headingRad = robot.getRotation().getRadians();

        double distPredM = getDistanceToTargetM(robotX, robotY, headingRad, predX, predY);
        double distActualM = getDistanceToTargetM(robotX, robotY, headingRad, actualTargetX, actualTargetY);

        double x_dist = distPredM * kMetersToInches;
        double y_dist = predZ * kMetersToInches - kTurretOffsetZ_in;
        double g = 386;
        double r = kFunnelRadiusIn * x_dist / (distActualM * kMetersToInches);
        double h = kFunnelHeightPlusAboveIn;
        double A1 = x_dist * x_dist;
        double B1 = x_dist;
        double D1 = y_dist;
        double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
        double B2 = -r;
        double D2 = h;
        double Bm = -B2 / B1;
        double A3 = Bm * A1 + A2;
        double D3 = Bm * D1 + D2;
        double a = D3 / A3;
        double b = (D1 - A1 * a) / B1;
        double theta = Math.atan(b);
        double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));

        if (Double.isNaN(v0) || Double.isNaN(theta)) {
            v0 = 0;
            theta = 0;
        }

        // v0 is in inches/sec — convert to rad/s via flywheel radius in inches
        double exitVelRadPerSec = v0 / kFlywheelRadiusIn;
        return new ShotData(exitVelRadPerSec, Math.PI / 2 - theta, new Translation3d(predX, predY, predZ));
    }

    // use an iterative lookahead approach to determine shot parameters for a moving robot
    public static ShotData iterativeMovingShotFromFunnelClearance(Pose2d robot,
            ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
        double robotX = robot.getX();
        double robotY = robot.getY();
        double headingRad = robot.getRotation().getRadians();
        double targetX = target.getX();
        double targetY = target.getY();
        double targetZ = target.getZ();
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;

        // Initial estimation (assuming unmoving robot)
        ShotData shot = calculateShotFromFunnelClearance(robot, target, target);

        double distM = getDistanceToTargetM(robotX, robotY, headingRad, targetX, targetY);
        double exitVelMps = shot.getExitVelocityMps();
        double tofSec = calculateTimeOfFlightSec(exitVelMps, shot.hoodAngle(), distM);

        double predX = targetX;
        double predY = targetY;

        for (int i = 0; i < iterations; i++) {
            // Inline predictTargetPos
            predX = targetX - vx * tofSec;
            predY = targetY - vy * tofSec;

            shot = calculateShotFromFunnelClearance(robot, targetX, targetY, predX, predY, targetZ);

            distM = getDistanceToTargetM(robotX, robotY, headingRad, predX, predY);
            exitVelMps = shot.getExitVelocityMps();
            tofSec = calculateTimeOfFlightSec(exitVelMps, shot.hoodAngle(), distM);
        }

        return shot;
    }

    /**
     * Use an iterative interpolation approach to determine shot parameters for a moving robot
     * compensating for speed, and the target.
     *
     * @param robot current robot pose
     * @param fieldSpeeds current robot speeds (w direction)
     * @param target target you are aiming for (either the passing point OR the HUB)
     * @param iterations amount of iterations to converge on one specific value
     * @return parameters to shoot a FUEL to the target accurately.
     */
    public static ShotData iterativeMovingShotFromInterpolationMap(Pose2d robot,
            ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {

        // Extract raw doubles once at entry
        double robotX = robot.getX();
        double robotY = robot.getY();
        double headingRad = robot.getRotation().getRadians();
        double targetX = target.getX();
        double targetY = target.getY();
        double targetZ = target.getZ();
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;

        double distance = getDistanceToTargetM(robotX, robotY, headingRad, targetX, targetY);
        ShotData shot = m_shotMap.get(distance);
        double exitVel = shot.exitVelocity();
        double hoodAngle = shot.hoodAngle();
        double tofSec = m_timeOfFlightMap.get(distance);

        double predX = targetX;
        double predY = targetY;

        //meant to iterate the process, and converge on one specific value.
        //gets a better ToF estimation & updates predictedTarget accordingly!
        for (int i = 0; i < iterations; i++) {
            double prevExitVel = exitVel;
            double prevHoodAngle = hoodAngle;
            double prevTOF = tofSec;
            double prevPredX = predX;
            double prevPredY = predY;

            // Inline predictTargetPos — no Translation3d/Time allocation
            predX = targetX - vx * tofSec;
            predY = targetY - vy * tofSec;

            distance = getDistanceToTargetM(robotX, robotY, headingRad, predX, predY);
            shot = m_shotMap.get(distance);
            exitVel = shot.exitVelocity();
            hoodAngle = shot.hoodAngle();
            tofSec = m_timeOfFlightMap.get(distance);

            double dExitVel = prevExitVel - exitVel;
            double dHood = prevHoodAngle - hoodAngle;
            double dTOF = prevTOF - tofSec;
            double dPredX = prevPredX - predX;
            double dPredY = prevPredY - predY;

            if (Math.abs(dHood) < .05 && Math.abs(dExitVel) < .05
                    && Math.sqrt(dPredX * dPredX + dPredY * dPredY) < .05
                    && Math.abs(dTOF) < .005) {
                break;
            }
        }

        return new ShotData(exitVel, hoodAngle, new Translation3d(predX, predY, targetZ));
    }

    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {
        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(RadiansPerSecond), hoodAngle.in(Radians), target);
        }

        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle) {
            this(exitVelocity, hoodAngle, FieldConstants.Hub.blueInnerCenterPoint);
        }

        public ShotData(double exitVelocity, double hoodAngle) {
            this(exitVelocity, hoodAngle, FieldConstants.Hub.blueInnerCenterPoint);
        }

        public ShotData minus(ShotData prevShotData) {
            double shotVelDiff = prevShotData.exitVelocity - this.exitVelocity;
            double shotHoodDiff = prevShotData.hoodAngle - this.hoodAngle;
            Translation3d shotTargetDiff = prevShotData.target.minus(this.target);

            return new ShotData(shotVelDiff, shotHoodDiff, shotTargetDiff);
        }

        public LinearVelocity getExitVelocity() {
            return angularToLinearVelocity(RadiansPerSecond.of(this.exitVelocity), kFlywheelRadius);
        }

        /** Raw double: exit velocity in m/s (no allocation). */
        public double getExitVelocityMps() {
            return angularToLinearVelocityMps(this.exitVelocity, kFlywheelRadiusM);
        }

        public Angle getHoodAngle() {
            return Radians.of(this.hoodAngle);
        }

        public Translation3d getTarget() {
            return this.target;
        }

        public static ShotData interpolate(ShotData start, ShotData end, double t) {
            return new ShotData(
                    MathUtil.interpolate(start.exitVelocity, end.exitVelocity, t),
                    MathUtil.interpolate(start.hoodAngle, end.hoodAngle, t),
                    end.target);
        }
    }
}
