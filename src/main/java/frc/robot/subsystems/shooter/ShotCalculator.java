
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.IntakeK.kLogTab;
import static frc.robot.Constants.ShooterK.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.FieldConstants;
import frc.util.WaltTunable;
import frc.util.WaltLogger.*;
import edu.wpi.first.units.measure.Time;

import java.util.DoubleSummaryStatistics;
import java.util.TreeMap;

public class ShotCalculator {
    private static final DoubleLogger log_distToTargetMeters = new DoubleLogger("Shooter/Calculator", "distTargetToMeters");
    private static final BooleanLogger log_isPassingLerp = new BooleanLogger("Shooter/Calculator", "isPassingLERP");
    private static final DoubleLogger log_dragCoefficient = new DoubleLogger("Shooter/Calculator", "dragCoefficient");
    private static final IntLogger log_lerpIterationCount = new IntLogger("Shooter/Calculator", "lerpIterationCount");
    private static final BooleanLogger log_calcConvergedBreakout = new BooleanLogger("Shooter/Calculator", "calcConvergedBreakout");

    private static final double kMetersToInches = 1.0 / 0.0254;
    // Horizontal drag damping: actual drift = v * (1 - e^(-c*t)) / c < v*t
    // c = 0 disables drag compensation. Enable via /ShotCalc/sotmDragCoeff/enabled.
    //YAYAYAYYAYAYAY 2974 IS OUR LUCKY NUMBER HUZZAH YAY YIPPEE
    private static final WaltTunable kDragCoeffTuner = new WaltTunable("/ShotCalc/sotmDragCoeff", 0.5000, false);
    // private static final Tracer m_iterativeTracer = new Tracer();

    // private static final double kRedHubCenterX = AllianceZoneUtil.redHubCenter.getX();
    // private static final double kBlueHubCenterX = AllianceZoneUtil.blueHubCenter.getX();

    private static final double[] kReductionDistances = {1.48, 2.31, 4.12};
    private static final double[] kReductionAmount = {0, 2, 4};

    private static final DoubleSummaryStatistics reductionSummaryStats = new DoubleSummaryStatistics();
    private static final DoubleSummaryStatistics distanceSummaryStats = new DoubleSummaryStatistics();

    private static final double minDistance;
    private static final double maxDistance;

    private static final boolean kRPSReductionNeeded = false;

    private static double kRPSBoost = 0.75;
    private static final WaltTunable kRPSBoostTuner = new WaltTunable("Shooter/Calculator/RPSBoost", kRPSBoost); 

    /**
     * Zero-allocation sorted-array interpolation tables replacing InterpolatingTreeMap.
     * All values stored in SI units (rad/s, radians, seconds).
     */
    public static final ShotLerpTable kShotTable;
    public static final ShotLerpTable kPassingTable;
    // public static final ShotLerpTable kAngryTurretTable;

    // this gets filled with distance scalars automatically based on all the distance keys in kShotTable
    public static final InterpolatingDoubleTreeMap kNewFuelAdjTable = new InterpolatingDoubleTreeMap();
    private static void addNewFuelAdjPoint(double distance) {
        kNewFuelAdjTable.put(distance, calcRPSReduction(distance));
    }

    static {
        //TODO: find the actual minDistance and maxDistance for shooting
        minDistance = 1.168;
        maxDistance = 5.672;

        kRPSBoost = kRPSBoostTuner.enabled() ? kRPSBoostTuner.get() : kRPSBoost;


        ShotLerpTable.Builder shot = new ShotLerpTable.Builder();

        // normal table
        // shot.add(8.627, 69.000, 1.160, 1.65, 0.500);
        // shot.add(7.801, 64.700, 1.134, 1.37, 0.500);
        // shot.add(6.973, 62.300, 1.104, 1.33, 0.500);
        // shot.add(6.126, 58.000, 1.071, 1.33, 0.500);
        // shot.add(5.577, 57.600, 1.046, 1.19, 0.500);
        // shot.add(4.555, 54.200, 0.994, 1.12, 0.500);
        // shot.add(4.231, 54.400, 0.974, 1.09, 0.500);
        // shot.add(3.798, 51.900, 0.852, 1.08, 0.500);
        // shot.add(3.267, 47.600, 0.907, 0.95, 0.500);
        // shot.add(2.826, 46.400, 0.869, 0.93, 0.500);
        // shot.add(2.212, 42.500, 0.608, 0.98, 0.500);
        // shot.add(1.929, 41.700, 0.500, 0.87, 0.500);
        // shot.add(1.093, 43.450, 0.000, 1.02, 0.500);
        // shot.add(0.985, 40.000, 0.000, 0.97, 0.500);

        //spoof table
        shot.add(8.627, 69.000, 1.160, 1.65, 0.500);
        shot.add(7.801, 65.865, 1.106, 1.37, 0.500); //1.524
        shot.add(6.973, 62.723, 1.046, 1.33, 0.500); //1.411
        shot.add(6.126, 59.509, 0.977, 1.33, 0.500); //1.308
        shot.add(5.577, 57.426, 0.927, 1.19, 0.500); //1.249
        shot.add(4.555, 53.548, 0.819, 1.12, 0.500); //1.153
        shot.add(4.231, 52.318, 0.779, 1.09, 0.500); //1.127
        shot.add(3.798, 50.675, 0.721, 1.08, 0.500); //1.095
        shot.add(3.267, 48.660, 0.641, 0.95, 0.500); //1.060
        shot.add(2.826, 46.986, 0.563, 0.93, 0.500); //1.035
        shot.add(2.212, 44.656, 0.432, 0.98, 0.500); //1.006
        shot.add(1.929, 43.582, 0.359, 0.87, 0.500); //0.995
        shot.add(1.093, 40.410, 0.056, 1.02, 0.500); //0.972
        shot.add(0.985, 40.000, 0.000, 0.97, 0.500);

        kShotTable = shot.build();
    }

    static {
        for (int i = 0; i < kReductionAmount.length; i++) {
            reductionSummaryStats.accept(kReductionAmount[i]);
            distanceSummaryStats.accept(kReductionDistances[i]);
        }

        for (int i = 0; i < kShotTable.keys.length; i++) {
            double dist = kShotTable.keys[i];
            addNewFuelAdjPoint(dist);
        }
    }

    static {
        ShotLerpTable.Builder passing = new ShotLerpTable.Builder();
        //---PASSING POINTS
        passing.add(4.0080, 48.000 + kRPSBoost, 0.70, 1.35, 0.254);
        passing.add(4.8160, 50.000 + kRPSBoost, 0.90, 1.29, 0.254);
        passing.add(5.0440, 51.000 + kRPSBoost, 1.00, 1.20, 0.254);
        passing.add(5.3520, 52.000 + kRPSBoost, 1.10, 1.15, 0.254);
        passing.add(5.6750, 54.000 + kRPSBoost, 1.15, 1.19, 0.254);
        passing.add(5.9900, 56.000 + kRPSBoost, 1.15, 1.27, 0.254);
        passing.add(6.3200, 58.000 + kRPSBoost, 1.15, 1.27, 0.254);
        passing.add(6.5830, 60.000 + kRPSBoost, 1.15, 1.30, 0.254);
        passing.add(6.8850, 62.000 + kRPSBoost, 1.15, 1.36, 0.254);
        passing.add(7.1690, 64.000 + kRPSBoost, 1.15, 1.41, 0.254);
        passing.add(7.5120, 66.000 + kRPSBoost, 1.15, 1.43, 0.254);
        passing.add(7.7780, 66.990 + kRPSBoost, 1.15, 1.46, 0.254);
        passing.add(8.1140, 67.750 + kRPSBoost, 1.15, 1.45, 0.254);
        passing.add(8.4420, 68.750 + kRPSBoost, 1.15, 1.58, 0.254);
        passing.add(8.7350, 70.250 + kRPSBoost, 1.15, 1.63, 0.254);
        passing.add(8.9970, 71.350 + kRPSBoost, 1.15, 1.71, 0.254);
        passing.add(10.419, 78.800 + kRPSBoost, 1.16, 1.78, 0.254);
        passing.add(10.722, 80.300 + kRPSBoost, 1.16, 1.80, 0.254);
        passing.add(11.076, 82.100 + kRPSBoost, 1.16, 1.79, 0.254);
        passing.add(11.367, 82.500 + kRPSBoost, 1.16, 1.87, 0.254);
        passing.add(11.722, 84.400 + kRPSBoost, 1.16, 1.85, 0.254);
        passing.add(12.060, 86.100 + kRPSBoost, 1.16, 1.87, 0.254);
        passing.add(12.358, 88.200 + kRPSBoost, 1.16, 1.92, 0.254);
        passing.add(12.670, 89.350 + kRPSBoost, 1.16, 1.95, 0.254);
        passing.add(13.048, 91.300 + kRPSBoost, 1.16, 1.92, 0.254);
        passing.add(13.053, 92.700 + kRPSBoost, 1.16, 2.04, 0.254);
        passing.add(13.657, 94.760 + kRPSBoost, 1.16, 2.02, 0.254);
        passing.add(14.020, 101.70 + kRPSBoost, 1.16, 2.00, 0.254);
        passing.add(14.355, 104.39 + kRPSBoost, 1.16, 2.08, 0.254);
        kPassingTable = passing.build();
    }
    static {
        //THIS IS ONLY USED IF THE TURRET IS IN A POSITION THAT IS UNABLE TO SHOOT WITH HOOD UP DURING PASSING
        //NO ANGRY PASSING IN OPPOSING ALLIANCE ZONE
        // ShotLerpTable.Builder angry = new ShotLerpTable.Builder();
        // angry.add(7.574, 92.5, 0.08, 2.08);
        // angry.add(7.135, 86, 0.08, 2.06);
        // angry.add(6.653, 81, 0.08, 2.04);
        // angry.add(6.254, 78, 0.08, 2.02);
        // //note that the points above have spoofed tofs
        // angry.add(5.672, 82.50 - kRPSReduction, 0.08, 2.11);
        // angry.add(5.321, 81.00 - kRPSReduction, 0.08, 1.94);
        // angry.add(5.223, 75.75 - kRPSReduction, 0.08, 1.79);
        // angry.add(5.167, 79.50 - kRPSReduction, 0.08, 2.03);
        // angry.add(4.996, 78.00 - kRPSReduction, 0.08, 1.97);
        // angry.add(4.869, 76.50 - kRPSReduction, 0.08, 1.80);
        // angry.add(4.696, 75.00 - kRPSReduction, 0.08, 1.33);
        // angry.add(4.546, 73.50 - kRPSReduction, 0.08, 1.83);
        // angry.add(4.402, 72.50 - kRPSReduction, 0.08, 1.85);
        // angry.add(4.258, 71.00 - kRPSReduction, 0.08, 1.64);
        // angry.add(4.098, 69.00 - kRPSReduction, 0.08, 1.58);
        // angry.add(3.932, 68.00 - kRPSReduction, 0.08, 1.71);
        // angry.add(3.785, 66.90 - kRPSReduction, 0.08, 1.56);
        // angry.add(3.611, 65.40 - kRPSReduction, 0.08, 1.56);
        // angry.add(3.464, 63.90 - kRPSReduction, 0.08, 1.50);
        // angry.add(3.293, 62.40 - kRPSReduction, 0.08, 1.50);
        // angry.add(3.134, 60.90 - kRPSReduction, 0.08, 1.51);
        // angry.add(2.995, 59.40 - kRPSReduction, 0.08, 1.37);
        // angry.add(2.855, 57.90 - kRPSReduction, 0.08, 1.35);
        // angry.add(2.704, 56.40 - kRPSReduction, 0.08, 1.38);
        // angry.add(2.586, 54.90 - kRPSReduction, 0.08, 1.28);
        // angry.add(2.395, 53.50 - kRPSReduction, 0.08, 1.29);
        // angry.add(2.221, 52.00 - kRPSReduction, 0.08, 1.25);
        // angry.add(2.083, 50.50 - kRPSReduction, 0.08, 1.14);
        // angry.add(1.912, 49.00 - kRPSReduction, 0.08, 1.19);
        // angry.add(1.691, 47.50 - kRPSReduction, 0.08, 0.98);
        // angry.add(1.575, 47.50 - kRPSReduction, 0.08, 1.18);
        // angry.add(1.528, 46.00 - kRPSReduction, 0.08, 1.20);
        // angry.add(1.307, 44.50 - kRPSReduction, 0.08, 1.13);
        // angry.add(1.168, 44.50 - kRPSReduction, 0.08, 1.16);
        // kAngryTurretTable = angry.build();
    }

    /**
     * @param distance
     * @return reduction magnitude
     */
    public static double calcRPSReduction(double distance) {
        double distanceSTDev = 0;
        double reductionSTDev = 0;
        double r = 0;

        for (int i = 0; i< kReductionAmount.length; i++) {
            distanceSTDev += Math.pow((kReductionAmount[i] - reductionSummaryStats.getAverage()),2);
            reductionSTDev += Math.pow((kReductionDistances[i] - distanceSummaryStats.getAverage()),2);
        }

        distanceSTDev = Math.sqrt(distanceSTDev/(kReductionDistances.length - 1));
        reductionSTDev = Math.sqrt(reductionSTDev/(kReductionAmount.length - 1));

        for (int i = 0; i < kReductionAmount.length; i++) {
            r += (((kReductionAmount[i] - reductionSummaryStats.getAverage())/reductionSTDev) * ((kReductionDistances[i] - distanceSummaryStats.getAverage())/reductionSTDev));
        }

        r /= (kReductionAmount.length - 1);
        
        double slope = r * (reductionSTDev/distanceSTDev);
        double intercept = reductionSummaryStats.getAverage() - slope * distanceSummaryStats.getAverage();

        return slope * distance + intercept;
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
    // ONLY USED FOR UNIT TEST!!!!!!!!!!!!!!!!!!!!!!!
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

    /** Returns drag-compensated drift time: (1 - e^(-c*t)) / c, or t if drag is disabled. */
    private static double dragCompensatedTOF(double tof, double dragCoeff) {
        // if (!kDragCoeffTuner.enabled()) return tof;
        double c = kDragCoeffTuner.enabled() ? kDragCoeffTuner.get() : dragCoeff;
        if (c < 1e-6) return tof;
        return (1.0 - Math.exp(-c * tof)) / c;
    }

    public static double getMinTimeOfFlight() {
        return kShotTable.tof(minDistance);
    }

    public static double getMaxTimeOfFlight() {
        return kShotTable.tof(maxDistance);
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
    public static ShotDataLerp iterativeMovingShotFromInterpolationMap(Pose2d robot,
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
        double omega = fieldSpeeds.omegaRadiansPerSecond;
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double turretX = robotX + kTurretOffsetX_m * cosH - kTurretOffsetY_m * sinH;
        double turretY = robotY + kTurretOffsetX_m * sinH + kTurretOffsetY_m * cosH;

        //turret pivot actual velocity
        double vxLaunch = vx - (turretY - robotY) * omega;
        double vyLaunch = vy + (turretX - robotX) * omega;

        double distance = getDistanceToTargetM(robotX, robotY, headingRad, targetX, targetY);
        boolean passing = ShooterCalc.isPassing().getAsBoolean();
        // boolean canTurretShoot = ShooterCalc.canTurretShoot();

        // ShotLerpTable shotTable = passing ? (canTurretShoot ? kPassingTable : kAngryTurretTable) : kShotTable;
        ShotLerpTable shotTable = passing ? kPassingTable : kShotTable;
        double exitVel = shotTable.exitVelocity(distance);
        double hoodAngle = shotTable.hoodAngle(distance);
        double tofSec = passing ? kPassingTable.tof(distance) : kShotTable.tof(distance);

        double predX = targetX;
        double predY = targetY;

        //meant to iterate the process, and converge on one specific value.
        //gets a better ToF estimation & updates predictedTarget accordingly!
        int iterCount = 0;
        boolean converged = false;
        for (int i = 0; i < iterations; i++) {
            iterCount = i + 1;
            double prevExitVel = exitVel;
            double prevHoodAngle = hoodAngle;
            double prevTOF = tofSec;
            double prevPredX = predX;
            double prevPredY = predY;

            // Inline predictTargetPos — no Translation3d/Time allocation
            double coeffDrag = 0.5000; //used for SOTM movement SIDE TO SIDE
            // double coeffDrag = shotTable.drag(distance);
            //2974 RAHHHHHHHHHHHHHHH – correction: more like 254 RAHHHHHHHHHHHHHHH
            // if ( (Math.abs(vx) <= 0.05) || (Math.abs(vy) <= 0.05) ) { //NOTE: not sure if these numbers are right
            //     coeffDrag = 0.2974; //used during static shot (or when the robot is low speed and should be static shooting)
            // }
            // if (distance >= 3.6) {
            //     coeffDrag = 0.53;   //0.7
            // }

            coeffDrag = kDragCoeffTuner.enabled() ? kDragCoeffTuner.get(): coeffDrag;
            double driftT = dragCompensatedTOF(tofSec, coeffDrag);
            predX = targetX - vxLaunch * driftT;
            predY = targetY - vyLaunch * driftT;

            distance = getDistanceToTargetM(robotX, robotY, headingRad, predX, predY);
            passing = ShooterCalc.isPassing().getAsBoolean();
            // shotTable = passing ? (canTurretShoot ? kPassingTable : kAngryTurretTable) : kShotTable;
            shotTable = passing ? kPassingTable : kShotTable;
            exitVel = shotTable.exitVelocity(distance);
            hoodAngle = shotTable.hoodAngle(distance);
            tofSec = passing ? kPassingTable.tof(distance) : kShotTable.tof(distance);

            double dExitVel = prevExitVel - exitVel;
            double dHood = prevHoodAngle - hoodAngle;
            double dTOF = prevTOF - tofSec;
            double dPredX = prevPredX - predX;
            double dPredY = prevPredY - predY;

            if (Math.abs(dHood) < .05 && Math.abs(dExitVel) < .5
                    && Math.sqrt(dPredX * dPredX + dPredY * dPredY) < .005
                    && Math.abs(dTOF) < .005) {
                converged = true;
                break;
            }
        }
        log_distToTargetMeters.accept(distance);
        log_isPassingLerp.accept(passing);
        log_calcConvergedBreakout.accept(converged);
        log_lerpIterationCount.accept(iterCount);
        ShotDataLerp data = new ShotDataLerp(exitVel, hoodAngle, new Translation3d(predX, predY, targetZ), tofSec);
        data.acceptLogging(data);
        return data;
    }

    public record ShotData (double exitVelocity, double hoodAngle, Translation3d target) {
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

    public record ShotDataLerp(double exitVelocity, double hoodAngle, Translation3d target, double tofSec) {
        public ShotDataLerp(ShotData data, double tofSec) {
            this(data.exitVelocity, data.hoodAngle, data.target, tofSec);
        }
        private static final String kCalcTab = "/ShotDataLerp";

        private static final DoubleLogger log_exitVelocity = new DoubleLogger(kLogTab + kCalcTab, "exitVelocity");
        private static final DoubleLogger log_hoodAngle = new DoubleLogger(kLogTab + kCalcTab, "hoodAngle");
        private static final Pose3dLogger log_target = new Pose3dLogger(kLogTab + kCalcTab, "target");
        private static final DoubleLogger log_tofSec = new DoubleLogger(kLogTab + kCalcTab, "tofSec");

        public void acceptLogging(ShotDataLerp data) {
            log_exitVelocity.accept(data.exitVelocity);
            log_hoodAngle.accept(data.hoodAngle);
            log_target.accept(data.target);
            log_tofSec.accept(data.tofSec);
        }

        public double getExitVelocity() { return exitVelocity; }
        public double getHoodAngle() { return hoodAngle; }
        public Translation3d getTarget() { return target; }
        public double getTofSec() { return tofSec; }
    }

    /**
     * Zero-allocation sorted-array interpolation table.
     * Replaces InterpolatingTreeMap / InterpolatingDoubleTreeMap on hot paths.
     * Values stored in SI units: exitVelocity in rad/s, hoodAngle in radians, tof in seconds.
     */
    public static final class ShotLerpTable {
        private final double[] keys;        // sorted ascending, meters
        private final double[] exitVels;    // rad/s
        private final double[] hoodAngles;  // radians
        private final double[] tofs;        // seconds
        private final double[] drags;

        private ShotLerpTable(double[] keys, double[] exitVels, double[] hoodAngles, double[] tofs, double[] drags) {
            this.keys = keys;
            this.exitVels = exitVels;
            this.hoodAngles = hoodAngles;
            this.tofs = tofs;
            this.drags = drags;
        }

        /** Interpolated exit velocity in rad/s. Zero allocation. */
        public double exitVelocity(double dist) { return lerp(dist, keys, exitVels); }
        /** Interpolated hood angle in radians. Zero allocation. */
        public double hoodAngle(double dist) { return lerp(dist, keys, hoodAngles); }
        /** Interpolated time of flight in seconds. Zero allocation. */
        public double tof(double dist) { return lerp(dist, keys, tofs); }
        /** Interpolated drag */
        public double drag(double dist) { return lerp(dist, keys, drags); }

        private static double lerp(double dist, double[] ks, double[] vs) {
            int n = ks.length;
            if (dist <= ks[0]) return vs[0];
            if (dist >= ks[n - 1]) return vs[n - 1];
            int lo = 0, hi = n - 1;
            while (hi - lo > 1) {
                int mid = (lo + hi) >>> 1;
                if (ks[mid] <= dist) lo = mid;
                else hi = mid;
            }
            double t = (dist - ks[lo]) / (ks[hi] - ks[lo]);
            return vs[lo] + t * (vs[hi] - vs[lo]);
        }

        public static final class Builder {
            private final TreeMap<Double, double[]> entries = new TreeMap<>();

            /** dist: meters, rps: rot/s (shooter), hoodRots: rotations, tof: seconds */
            public void add(double dist, double rps, double hoodRots, double tof, double drag) {
                entries.put(dist, new double[]{
                    rps * (2.0 * Math.PI),        // rot/s → rad/s
                    hoodRots * (2.0 * Math.PI),   // rotations → radians
                    tof,
                    drag
                });
            }

            public ShotLerpTable build() {
                int n = entries.size();
                double[] ks = new double[n];
                double[] evs = new double[n];
                double[] has = new double[n];
                double[] ts = new double[n];
                double[] ds = new double[n];
                int i = 0;
                for (var e : entries.entrySet()) {
                    ks[i] = e.getKey();
                    double[] v = e.getValue();
                    evs[i] = v[0] - (kRPSReductionNeeded ? kNewFuelAdjTable.get(v[0]) : 0);
                    has[i] = v[1];
                    ts[i] = v[2];
                    ds[i] = v[3];
                    i++;
                }
                return new ShotLerpTable(ks, evs, has, ts, ds);
            }
        }
    }
}
