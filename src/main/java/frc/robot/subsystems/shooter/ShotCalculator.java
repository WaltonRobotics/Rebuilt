
package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.InchesPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;
import static frc.robot.Constants.IntakeK.kLogTab;
import static frc.robot.Constants.ShooterK.*;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import frc.robot.FieldConstants;
import frc.util.WaltTunable;
import frc.util.WaltLogger.*;
import org.wpilib.units.measure.Time;

import java.util.DoubleSummaryStatistics;
import java.util.TreeMap;

public class ShotCalculator {
    private static final DoubleLogger log_distToTargetMeters = new DoubleLogger("Shooter/Calculator", "distTargetToMeters");
    private static final BooleanLogger log_isPassingLerp = new BooleanLogger("Shooter/Calculator", "isPassingLERP");
    private static final DoubleLogger log_dragCoefficient = new DoubleLogger("Shooter/Calculator", "dragCoefficient");
    private static final IntLogger log_lerpIterationCount = new IntLogger("Shooter/Calculator", "lerpIterationCount");
    private static final BooleanLogger log_calcConvergedBreakout = new BooleanLogger("Shooter/Calculator", "calcConvergedBreakout");
    private static final DoubleLogger log_shotConfidence = new DoubleLogger("Shooter/Calculator", "shotConfidence");
    private static final DoubleLogger log_solverQuality = new DoubleLogger("Shooter/Calculator", "solverQuality");

    private static final double kMetersToInches = 1.0 / 0.0254;
    // Horizontal drag damping: actual drift = v * (1 - e^(-c*t)) / c < v*t
    // c = 0 disables drag compensation. Enable via /ShotCalc/sotmDragCoeff/enabled.
    private static final WaltTunable kDragCoeffTuner = new WaltTunable("/ShotCalc/sotmDragCoeff", 0.24, false);

    // if we're moving slower than this, just treat it as a static shot (no SOTM)
    private static final double kMinSOTMSpeed = 0.1; // m/s
    // if we're moving faster than this, SOTM gets unreliable — cap it
    private static final double kMaxSOTMSpeed = 3.5; // m/s

    // how much latency we're compensating for in our pose prediction
    // vision pipeline delay + network round-trip
    private static final double kPhaseDelayMs = 30.0;
    // mechanical delay (hood/flywheel settling time)
    private static final double kMechLatencyMs = 20.0;
    // total latency we predict ahead by, in seconds
    private static final double kTotalLatencySec = (kPhaseDelayMs + kMechLatencyMs) / 1000.0;

    // Newton-Raphson solver tuning — shouldn't need to touch these often
    private static final int kMaxNewtonIterations = 25; // way more than it should ever need (usually 2-3)
    private static final double kNewtonConvergenceTol = 0.001; // how close TOF needs to be between iterations
    private static final double kTofMin = 0.05; // sanity clamp — can't have negative or near-zero TOF
    private static final double kTofMax = 5.0;  // 5 seconds is absurdly long, something is wrong if we hit this
    private static final double kTofDerivH = 0.001; // step size for numerical derivative of the TOF lookup table

    // confidence scoring weights — controls how much each factor matters
    // higher weight = that factor has more influence on overall confidence
    private static final double kWConvergence = 1.0;       // did the solver actually converge?
    private static final double kWVelocityStability = 0.8;  // are we accelerating/decelerating erratically?
    private static final double kWHeadingAccuracy = 1.5;    // is the turret pointing where we want? (most important)
    private static final double kWDistanceInRange = 0.5;    // are we in a reasonable scoring range?
    private static final double kHeadingMaxErrorRad = Math.toRadians(15); // max heading error before confidence tanks
    private static final double kHeadingSpeedScalar = 1.0;      // how much speed tightens the heading tolerance
    private static final double kHeadingReferenceDistance = 2.5; // reference dist for scaling heading tolerance

    // stashed values from last cycle — used to estimate acceleration and warm-start the solver
    private static double s_prevVx = 0;
    private static double s_prevVy = 0;
    private static double s_prevOmega = 0;
    private static double s_prevSpeed = 0;
    private static double s_prevTof = -1;     // last cycle's solved TOF, -1 = never solved yet
    private static double s_prevRawDist = -1;  // last cycle's raw distance to target, -1 = uninitialized
    // private static final Tracer m_iterativeTracer = new Tracer();

    // private static final double kRedHubCenterX = AllianceZoneUtil.redHubCenter.getX();
    // private static final double kBlueHubCenterX = AllianceZoneUtil.blueHubCenter.getX();

    private static final double[] kReductionDistances = {1.48, 2.31, 4.12};
    private static final double[] kReductionAmount = {0, 2, 4};

    private static final DoubleSummaryStatistics reductionSummaryStats = new DoubleSummaryStatistics();
    private static final DoubleSummaryStatistics distanceSummaryStats = new DoubleSummaryStatistics();

    private static final double minScoringDistance;
    private static final double maxScoringDistance;

    private static final boolean kRPSReductionNeeded = false;

    private static double kRPSBoost = 0.75;
    private static double kLongRangeRPSBoost = 0.35;

    private static double kScoringRPSBoost = -0.2;
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
        minScoringDistance = 0.985;
        maxScoringDistance = 8.627;

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
        shot.add(8.627, 69.000 + kScoringRPSBoost, 1.160, 1.65, 0.500);
        shot.add(7.801, 65.865 + kScoringRPSBoost, 1.106, 1.37, 0.500); //1.524
        shot.add(6.973, 62.723 + kScoringRPSBoost, 1.046, 1.33, 0.500); //1.411
        shot.add(6.126, 59.509 + kScoringRPSBoost, 0.977, 1.33, 0.500); //1.308
        shot.add(5.577, 57.426 + kScoringRPSBoost, 0.927, 1.19, 0.500); //1.249
        shot.add(4.555, 53.548 + kScoringRPSBoost, 0.819, 1.12, 0.500); //1.153
        shot.add(4.231, 52.318 + kScoringRPSBoost, 0.779, 1.09, 0.500); //1.127
        shot.add(3.798, 50.675 + kScoringRPSBoost, 0.721, 1.08, 0.500); //1.095
        shot.add(3.267, 48.660 + kScoringRPSBoost, 0.641, 0.95, 0.500); //1.060
        shot.add(2.826, 46.986 + kScoringRPSBoost, 0.563, 0.93, 0.500); //1.035
        shot.add(2.212, 44.656 + kScoringRPSBoost, 0.432, 0.98, 0.500); //1.006
        shot.add(1.929, 43.582 + kScoringRPSBoost, 0.359, 0.87, 0.500); //0.995
        shot.add(1.093, 40.410 + kScoringRPSBoost, 0.056, 1.02, 0.500); //0.972
        shot.add(0.985, 40.000 + kScoringRPSBoost, 0.000, 0.97, 0.500);

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
        passing.add(10.419, 78.800 + kRPSBoost + kLongRangeRPSBoost, 1.16, 1.78, 0.254);
        passing.add(10.722, 80.300 + kRPSBoost + kLongRangeRPSBoost, 1.16, 1.80, 0.254);
        passing.add(11.076, 82.100 + kRPSBoost + kLongRangeRPSBoost, 1.16, 1.79, 0.254);
        passing.add(11.367, 82.500 + kRPSBoost + kLongRangeRPSBoost, 1.16, 1.87, 0.254);
        passing.add(11.722, 84.400 + kRPSBoost + kLongRangeRPSBoost, 1.16, 1.85, 0.254);
        passing.add(12.060, 86.100 + kRPSBoost + kLongRangeRPSBoost, 1.16, 1.87, 0.254);
        passing.add(12.358, 88.200 + kRPSBoost + kLongRangeRPSBoost, 1.16, 1.92, 0.254);
        passing.add(12.670, 89.350 + kRPSBoost + kLongRangeRPSBoost, 1.16, 1.95, 0.254);
        passing.add(13.048, 91.300 + kRPSBoost + kLongRangeRPSBoost, 1.16, 1.92, 0.254);
        passing.add(13.053, 92.700 + kRPSBoost + kLongRangeRPSBoost, 1.16, 2.04, 0.254);
        passing.add(13.657, 94.760 + kRPSBoost + kLongRangeRPSBoost, 1.16, 2.02, 0.254);
        passing.add(14.020, 101.70 + kRPSBoost + kLongRangeRPSBoost, 1.16, 2.00, 0.254);
        passing.add(14.355, 104.39 + kRPSBoost + kLongRangeRPSBoost, 1.16, 2.08, 0.254);
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
        return kShotTable.tof(minScoringDistance);
    }

    public static double getMaxTimeOfFlight() {
        return kShotTable.tof(maxScoringDistance);
    }

    /**
     * Predicts where the robot WILL be by the time the shot actually leaves the barrel.
     * Our pose data is stale by ~50ms (vision pipeline + mechanical response), so we
     * extrapolate forward using velocity AND acceleration (2nd-order, not just linear).
     *
     * Acceleration is estimated by comparing this cycle's velocity to last cycle's (20ms apart).
     * This is the same idea as 4322's latency compensation.
     *
     * Math: compensatedPose = rawPose.exp(v*dt + 0.5*a*dt^2)
     */
    public static Pose2d compensatePoseForLatency(Pose2d rawPose, ChassisVelocities speeds) {
        double dt = kTotalLatencySec;

        // estimate acceleration by comparing velocity to last cycle (finite difference over 20ms)
        double ax = (speeds.vx - s_prevVx) / 0.02;
        double ay = (speeds.vy - s_prevVy) / 0.02;
        double aOmega = (speeds.omega - s_prevOmega) / 0.02;

        // 2nd order extrapolation: position += v*dt + 0.5*a*dt^2
        Pose2d compensated = rawPose.plus(new Twist2d(
            speeds.vx * dt + 0.5 * ax * dt * dt,
            speeds.vy * dt + 0.5 * ay * dt * dt,
            speeds.omega * dt + 0.5 * aOmega * dt * dt).exp());

        // stash for next cycle so we can compute acceleration again
        s_prevVx = speeds.vx;
        s_prevVy = speeds.vy;
        s_prevOmega = speeds.omega;

        return compensated;
    }

    // numerically approximates how TOF changes as distance changes in the lookup table
    // we need this derivative for the Newton solver's update step
    // central finite difference: nudge distance up and down by a tiny amount, see what TOF does
    private static double tofMapDerivative(ShotLerpTable table, double dist) {
        double h = kTofDerivH;
        return (table.tof(dist + h) - table.tof(dist - h)) / (2.0 * h);
    }

    /**
     * Gives us a 0-100 confidence score for the current shot.
     * Right now this is ONLY for logging/telemetry — does NOT gate firing.
     * Could be used to gate later if we want to.
     *
     * Uses a weighted geometric mean of 4 factors, so if ANY factor is zero
     * the whole confidence goes to zero (which makes sense — if the solver
     * didn't converge, we shouldn't trust the shot no matter what).
     *
     * Inspired by 4322's approach.
     */
    public static double computeShotConfidence(
            double solverQuality, double currentSpeed,
            double headingErrorRad, double distance) {

        // 1) did the newton solver actually converge? if not, we don't trust the aim point
        double convergenceQuality = Math.clamp(solverQuality, 0, 1);

        // 2) are we changing speed rapidly? if speed is jumping around, the predicted
        //    aim point is gonna be jittery — penalize that
        double speedDelta = Math.abs(currentSpeed - s_prevSpeed);
        double velocityStability = Math.clamp(1.0 - speedDelta / 0.5, 0, 1);
        s_prevSpeed = currentSpeed;

        // 3) is the turret actually pointing where we want it to?
        //    tolerance gets TIGHTER when we're going fast or close to the target
        //    (makes sense — small angular error matters more up close and at speed)
        double distanceScale = Math.clamp(kHeadingReferenceDistance / Math.max(distance, 0.1), 0.5, 2.0);
        double speedScale = 1.0 / (1.0 + kHeadingSpeedScalar * currentSpeed);
        double scaledMaxError = kHeadingMaxErrorRad * distanceScale * speedScale;
        double headingErr = Math.abs(headingErrorRad);
        double headingAccuracy = Math.clamp(1.0 - headingErr / scaledMaxError, 0, 1);

        // 4) are we in a reasonable shooting range? confidence peaks in the middle of our
        //    interpolation table range and drops off toward the edges
        double rangeSpan = maxScoringDistance - minScoringDistance;
        double rangeFraction = (distance - minScoringDistance) / rangeSpan;
        double distInRange = 1.0 - 2.0 * Math.abs(rangeFraction - 0.5);
        distInRange = Math.clamp(distInRange, 0, 1);

        // weighted geometric mean — multiply factors together (in log space) with weights
        // this way one bad factor drags everything down proportionally
        double[] c = {convergenceQuality, velocityStability, headingAccuracy, distInRange};
        double[] w = {kWConvergence, kWVelocityStability, kWHeadingAccuracy, kWDistanceInRange};

        double sumW = 0;
        double logSum = 0;
        for (int i = 0; i < c.length; i++) {
            if (c[i] <= 0) return 0; // any zero factor = zero confidence, full stop
            logSum += w[i] * Math.log(c[i]);
            sumW += w[i];
        }
        if (sumW <= 0) return 0;
        double composite = Math.exp(logSum / sumW) * 100.0;
        return Math.clamp(composite, 0, 100);
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
    public static Translation3d predictTargetPos(Translation3d target, ChassisVelocities fieldSpeeds, Time timeOfFlight) {
        //need time of flight b/c that tells you how close/far you can shoot to the target according to speeds.
        double predictedX = target.getX() - fieldSpeeds.vx * timeOfFlight.in(Seconds); 
        double predictedY = target.getY() - fieldSpeeds.vy * timeOfFlight.in(Seconds);

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
            ChassisVelocities fieldSpeeds, Translation3d target, int iterations) {
        double robotX = robot.getX();
        double robotY = robot.getY();
        double headingRad = robot.getRotation().getRadians();
        double targetX = target.getX();
        double targetY = target.getY();
        double targetZ = target.getZ();
        double vx = fieldSpeeds.vx;
        double vy = fieldSpeeds.vy;

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
     * The main SOTM solver. Uses Newton-Raphson instead of the old fixed-point iteration.
     *
     * The problem: we need to find a TOF (time of flight) where the aim point we compute
     * FROM that TOF gives us a distance that, when we look up in the shot table, gives us
     * back the SAME TOF. It's a chicken-and-egg problem — this solver finds the answer
     * where both sides agree.
     *
     * Newton-Raphson converges in 2-3 iterations instead of 8, and gives us a derivative
     * for free so we know how confident we are in the answer.
     *
     * Also warm-starts from last cycle's TOF (if the robot hasn't moved too far) so most
     * cycles it barely has to do any work.
     */
    public static ShotDataLerp iterativeMovingShotFromInterpolationMap(Pose2d robot,
            ChassisVelocities fieldSpeeds, Translation3d target, int iterations) {

        // pull everything into raw doubles up front so we're not calling getters in the loop
        double robotX = robot.getX();
        double robotY = robot.getY();
        double headingRad = robot.getRotation().getRadians();
        double targetX = target.getX();
        double targetY = target.getY();
        double targetZ = target.getZ();
        double vx = fieldSpeeds.vx;
        double vy = fieldSpeeds.vy;
        double omega = fieldSpeeds.omega;
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double turretX = robotX + kTurretOffsetX_m * cosH - kTurretOffsetY_m * sinH;
        double turretY = robotY + kTurretOffsetX_m * sinH + kTurretOffsetY_m * cosH;

        // the turret isn't at the robot center — it's offset, so when the robot spins
        // the turret pivot has its own tangential velocity on top of the robot's translation
        double vxLaunch = vx - (turretY - robotY) * omega;
        double vyLaunch = vy + (turretX - robotX) * omega;

        // if we're barely moving, just treat it as a static shot — SOTM compensation
        // at near-zero speeds just adds noise
        // TODO: see if this needs to be gated on the higher end
        double speed = Math.hypot(vxLaunch, vyLaunch);
        boolean velocityFiltered = speed < kMinSOTMSpeed; //|| speed > kMaxSOTMSpeed;
        if (velocityFiltered) {
            vxLaunch = 0;
            vyLaunch = 0;
        }

        // vector from where the turret is to where the target is
        double rx = targetX - turretX;
        double ry = targetY - turretY;
        double rawDistance = Math.hypot(rx, ry);

        boolean passing = ShooterCalc.isPassing().getAsBoolean();
        ShotLerpTable shotTable = passing ? kPassingTable : kShotTable;

        double dragCoeff = kDragCoeffTuner.enabled() ? kDragCoeffTuner.get() : 0.50;

        // --- NEWTON-RAPHSON SOLVER ---
        // try to warm-start from last cycle's TOF — but only if we haven't moved too far.
        // if the robot teleported or the target changed, the old TOF is garbage so we cold-start
        // from the lookup table instead.
        boolean warmStartValid = s_prevTof >= 0
            && s_prevRawDist >= 0
            && Math.abs(rawDistance - s_prevRawDist) < 0.5; // <0.5m change per cycle = sane
        double tof = warmStartValid ? s_prevTof : shotTable.tof(rawDistance);
        tof = Math.clamp(tof, kTofMin, kTofMax);

        double projDist = rawDistance;
        int iterationsUsed = 0;
        boolean converged = false;
        double solverResidual = 1.0;

        for (int i = 0; i < kMaxNewtonIterations; i++) {
            double prevTOF = tof;

            // how far the game piece drifts horizontally due to air drag
            // drag makes the effective drift time shorter than the actual TOF
            double c = dragCoeff;
            double dragExp = c < 1e-6 ? 1.0 : Math.exp(-c * tof);
            double driftTOF = c < 1e-6 ? tof : (1.0 - dragExp) / c;

            // where we need to aim — offset the target by how far the robot will drift
            // during the time the ball is in the air
            double prx = rx - vxLaunch * driftTOF;
            double pry = ry - vyLaunch * driftTOF;
            projDist = Math.hypot(prx, pry);

            // edge case: we're basically ON TOP of the target, math falls apart
            if (projDist < 0.01) {
                tof = shotTable.tof(rawDistance);
                iterationsUsed = kMaxNewtonIterations + 1;
                break;
            }

            // look up what TOF the shot table says for this projected distance
            double lookupTOF = shotTable.tof(projDist);

            // newton step — we're solving f(tof) = lookupTOF(projDist(tof)) - tof = 0
            // the derivative comes from the chain rule:
            //   f'(tof) = (d/d_tof of lookupTOF) * (d/d_tof of projDist) - 1
            // projDist changes with tof because the drift changes, which moves the aim point
            double dPrime = -dragExp * (prx * vxLaunch + pry * vyLaunch) / projDist;
            double gPrime = tofMapDerivative(shotTable, projDist);
            double f = lookupTOF - tof;
            double fPrime = gPrime * dPrime - 1.0;

            if (Math.abs(fPrime) > 0.01) {
                // normal newton update: tof_new = tof - f/f'
                tof = tof - f / fPrime;
            } else {
                // derivative is basically flat — just use the lookup value directly
                // (this is the old fixed-point approach as a fallback)
                tof = lookupTOF;
            }

            tof = Math.clamp(tof, kTofMin, kTofMax);
            iterationsUsed = i + 1;
            solverResidual = Math.abs(tof - prevTOF);

            if (solverResidual < kNewtonConvergenceTol) {
                converged = true;
                break;
            }
        }

        // save for next cycle's warm-start
        s_prevTof = tof;
        s_prevRawDist = rawDistance;

        // how good was the solve? 1.0 = perfect convergence, drops toward 0 if it didn't settle
        double solverQuality = converged ? 1.0
            : Math.clamp(1.0 - solverResidual / 0.1, 0, 1);
        log_solverQuality.accept(solverQuality);

        // now that we have a converged TOF, compute where we actually need to aim
        double finalDriftT = dragCompensatedTOF(tof, dragCoeff);
        double predX = targetX - vxLaunch * finalDriftT;
        double predY = targetY - vyLaunch * finalDriftT;
        double distance = getDistanceToTargetM(robotX, robotY, headingRad, predX, predY);

        // look up the actual shot parameters for our final aimed distance
        passing = ShooterCalc.isPassing().getAsBoolean();
        shotTable = passing ? kPassingTable : kShotTable;
        double exitVel = shotTable.exitVelocity(distance);
        double hoodAngle = shotTable.hoodAngle(distance);
        double tofSec = shotTable.tof(distance);

        log_distToTargetMeters.accept(distance);
        log_isPassingLerp.accept(passing);
        log_calcConvergedBreakout.accept(converged);
        log_lerpIterationCount.accept(iterationsUsed);
        ShotDataLerp data = new ShotDataLerp(exitVel, hoodAngle, new Translation3d(predX, predY, targetZ), tofSec, solverQuality);
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
                    MathUtil.lerp(start.exitVelocity, end.exitVelocity, t),
                    MathUtil.lerp(start.hoodAngle, end.hoodAngle, t),
                    end.target);
        }
    }

    // solverQuality gets piped into the confidence scoring system
    public record ShotDataLerp(double exitVelocity, double hoodAngle, Translation3d target, double tofSec, double solverQuality) {
        // old callers that don't care about solver quality just get 1.0 (fully confident)
        public ShotDataLerp(double exitVelocity, double hoodAngle, Translation3d target, double tofSec) {
            this(exitVelocity, hoodAngle, target, tofSec, 1.0);
        }
        public ShotDataLerp(ShotData data, double tofSec) {
            this(data.exitVelocity, data.hoodAngle, data.target, tofSec, 1.0);
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
        public double getSolverQuality() { return solverQuality; }
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
