package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;

import frc.robot.Constants.ShooterK;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShotCalculator.ShotData;

import org.junit.jupiter.api.*;

/**
 * Performance benchmark comparing three approaches for the shot calculation hot path:
 *   1. Immutable WPILib Units (current implementation)
 *   2. Mutable WPILib Units (MutAngle, MutDistance, etc.)
 *   3. Raw doubles (no Units library at all)
 *
 * Each approach is timed over many iterations of the full calcShot pipeline.
 * Results are printed as a comparison table.
 */
class ShotCalcPerfTest {

    private static Translation3d HUB_TARGET;
    private static Pose2d MID_POSE;
    private static final ChassisSpeeds MOVING_SPEEDS = new ChassisSpeeds(1.5, -0.5, 0.3);

    // Precomputed constants as raw doubles (mirrors Constants.ShooterK)
    private static final double kTurretOffsetX_m = edu.wpi.first.math.util.Units.inchesToMeters(-4.744);
    private static final double kTurretOffsetY_m = edu.wpi.first.math.util.Units.inchesToMeters(-4.239);
    private static final double kTurretOffsetZ_m = edu.wpi.first.math.util.Units.inchesToMeters(17.260);
    private static final double kTurretAngleOffsetRad = Math.toRadians(-135);
    private static final double kFlywheelRadiusM = edu.wpi.first.math.util.Units.inchesToMeters(1.5);
    private static final double kGravityInPerSec2 = 9.81 * 39.3701; // m/s^2 -> in/s^2
    private static final double kTurretMinRotsD = -0.75;
    private static final double kTurretMaxRotsD = 0.75;

    // LERP map data as raw double arrays, SORTED BY DISTANCE (must match TreeMap ordering)
    // Original entries: 1.476, 3.763, 3.854, 3.903, 5.657
    private static final double[] LERP_DISTANCES = {1.476, 3.763, 3.854, 3.903, 5.657};
    private static final double[] LERP_EXIT_VEL;
    private static final double[] LERP_HOOD_ANGLE;
    private static final double[] LERP_TOF = {1.125, 1.43, 1.38, 1.35, 1.56};

    static {
        // Convert the LERP map entries to raw radians, in sorted distance order
        LERP_EXIT_VEL = new double[] {
            RotationsPerSecond.of(57.72).in(RadiansPerSecond),  // 1.476m
            RotationsPerSecond.of(77).in(RadiansPerSecond),     // 3.763m
            RotationsPerSecond.of(79).in(RadiansPerSecond),     // 3.854m
            RotationsPerSecond.of(73.18).in(RadiansPerSecond),  // 3.903m
            RotationsPerSecond.of(95.00).in(RadiansPerSecond),  // 5.657m
        };
        LERP_HOOD_ANGLE = new double[] {
            Degrees.of(150.77).in(Radians),  // 1.476m
            Degrees.of(300).in(Radians),     // 3.763m
            Degrees.of(345).in(Radians),     // 3.854m
            Degrees.of(327.75).in(Radians),  // 3.903m
            Degrees.of(374.20).in(Radians),  // 5.657m
        };
    }

    @BeforeAll
    static void setup() {
        HAL.initialize(500, 0);
        HUB_TARGET = FieldConstants.Hub.blueInnerCenterPoint;
        double hubX = HUB_TARGET.getX();
        double hubY = HUB_TARGET.getY();
        MID_POSE = new Pose2d(hubX - 3.9, hubY + 1.0, Rotation2d.fromDegrees(15));
    }

    // ================================================================
    //  Approach 1: Immutable Units (current implementation)
    // ================================================================

    private static ShotData immutableCalcShot(Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target) {
        return ShotCalculator.iterativeMovingShotFromInterpolationMap(robot, fieldSpeeds, target, 3);
    }

    // ================================================================
    //  Approach 2: Mutable Units
    //  Reuses MutDistance, MutTime, etc. to avoid allocations in the loop.
    // ================================================================

    // Preallocated mutable measures for the mutable approach
    private static final MutDistance mut_dist = Meters.of(0).mutableCopy();
    private static final MutTime mut_tof = Seconds.of(0).mutableCopy();
    private static final MutLinearVelocity mut_exitVel = MetersPerSecond.of(0).mutableCopy();
    private static final MutAngle mut_hoodAngle = Radians.of(0).mutableCopy();

    private static double mutableGetDistanceToTarget(Pose2d robot, Translation3d target) {
        Pose3d turretPose = new Pose3d(robot).transformBy(ShooterK.kTurretTransform);
        double dist = turretPose.getTranslation().toTranslation2d().getDistance(target.toTranslation2d());
        mut_dist.mut_replace(dist, Meters);
        return dist;
    }

    private static ShotData mutableIterativeMovingShot(Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target) {
        double distance = mutableGetDistanceToTarget(robot, target);
        ShotData shot = ShotCalculator.m_shotMap.get(distance);
        shot = new ShotData(shot.exitVelocity(), shot.hoodAngle(), target);

        // Use the mutable TOF instead of allocating
        double tofSec = ShotCalculator.m_timeOfFlightMap.get(distance);
        mut_tof.mut_replace(tofSec, Seconds);

        Translation3d predictedTarget = target;

        for (int i = 0; i < 3; i++) {
            ShotData prevShot = shot;
            double prevTOF = tofSec;
            Translation3d prevPredTarget = predictedTarget;

            // predictTargetPos inline using mutable TOF
            double predX = target.getX() - fieldSpeeds.vxMetersPerSecond * tofSec;
            double predY = target.getY() - fieldSpeeds.vyMetersPerSecond * tofSec;
            predictedTarget = new Translation3d(predX, predY, target.getZ());

            distance = mutableGetDistanceToTarget(robot, predictedTarget);
            shot = ShotCalculator.m_shotMap.get(distance);
            shot = new ShotData(shot.exitVelocity(), shot.hoodAngle(), predictedTarget);
            tofSec = ShotCalculator.m_timeOfFlightMap.get(distance);
            mut_tof.mut_replace(tofSec, Seconds);

            ShotData shotDiff = shot.minus(prevShot);
            double tofDiff = prevTOF - tofSec;

            if (shotDiff.hoodAngle() < .05 && shotDiff.exitVelocity() < .05
                    && prevShot.target().getDistance(shot.getTarget()) < .05
                    && Math.abs(tofDiff) < .005
                    && prevPredTarget.getDistance(predictedTarget) < .05) {
                break;
            }
        }
        return shot;
    }

    // ================================================================
    //  Approach 3: Raw Doubles (zero Units library usage)
    // ================================================================

    /** Linear interpolation between LERP table entries, all in raw doubles. */
    private static double lerpLookup(double dist, double[] values) {
        if (dist <= LERP_DISTANCES[0]) return values[0];
        if (dist >= LERP_DISTANCES[LERP_DISTANCES.length - 1]) return values[values.length - 1];
        for (int i = 0; i < LERP_DISTANCES.length - 1; i++) {
            if (dist <= LERP_DISTANCES[i + 1]) {
                double t = (dist - LERP_DISTANCES[i]) / (LERP_DISTANCES[i + 1] - LERP_DISTANCES[i]);
                return values[i] + t * (values[i + 1] - values[i]);
            }
        }
        return values[values.length - 1];
    }

    /**
     * Turret-transformed distance to target in meters, all raw doubles.
     * Avoids Pose3d/Transform3d allocations entirely.
     */
    private static double rawGetDistanceToTarget(double robotX, double robotY, double robotHeadingRad,
            double targetX, double targetY) {
        // Apply turret transform: rotate offset by robot heading, then translate
        double cosH = Math.cos(robotHeadingRad + kTurretAngleOffsetRad);
        double sinH = Math.sin(robotHeadingRad + kTurretAngleOffsetRad);
        // Turret offset in field frame (rotate the local offset by robot heading + turret angle offset)
        double cosR = Math.cos(robotHeadingRad);
        double sinR = Math.sin(robotHeadingRad);
        double turretX = robotX + kTurretOffsetX_m * cosR - kTurretOffsetY_m * sinR;
        double turretY = robotY + kTurretOffsetX_m * sinR + kTurretOffsetY_m * cosR;
        double dx = turretX - targetX;
        double dy = turretY - targetY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Full iterative moving shot pipeline using only raw doubles.
     * Returns [exitVelocity_radPerSec, hoodAngle_rad, predTargetX, predTargetY, predTargetZ].
     */
    private static double[] rawIterativeMovingShot(double robotX, double robotY, double robotHeadingRad,
            double vxMps, double vyMps, double targetX, double targetY, double targetZ) {

        double distance = rawGetDistanceToTarget(robotX, robotY, robotHeadingRad, targetX, targetY);
        double exitVel = lerpLookup(distance, LERP_EXIT_VEL);
        double hoodAngle = lerpLookup(distance, LERP_HOOD_ANGLE);
        double tofSec = lerpLookup(distance, LERP_TOF);

        double predX = targetX;
        double predY = targetY;

        double prevExitVel, prevHoodAngle, prevTOF, prevPredX, prevPredY;

        for (int i = 0; i < 3; i++) {
            prevExitVel = exitVel;
            prevHoodAngle = hoodAngle;
            prevTOF = tofSec;
            prevPredX = predX;
            prevPredY = predY;

            predX = targetX - vxMps * tofSec;
            predY = targetY - vyMps * tofSec;

            distance = rawGetDistanceToTarget(robotX, robotY, robotHeadingRad, predX, predY);
            exitVel = lerpLookup(distance, LERP_EXIT_VEL);
            hoodAngle = lerpLookup(distance, LERP_HOOD_ANGLE);
            tofSec = lerpLookup(distance, LERP_TOF);

            double dExitVel = prevExitVel - exitVel;
            double dHood = prevHoodAngle - hoodAngle;
            double dPredX = prevPredX - predX;
            double dPredY = prevPredY - predY;
            double dTOF = prevTOF - tofSec;

            if (Math.abs(dHood) < .05 && Math.abs(dExitVel) < .05
                    && Math.sqrt(dPredX * dPredX + dPredY * dPredY) < .05
                    && Math.abs(dTOF) < .005) {
                break;
            }
        }

        return new double[]{exitVel, hoodAngle, predX, predY, targetZ};
    }

    /**
     * Full calcAzimuth equivalent, raw doubles only.
     * Returns [turretReferenceRots, turretFFRadPerSec].
     */
    private static double[] rawCalcAzimuth(double targetX, double targetY, double targetZ,
            double robotX, double robotY, double robotHeadingRad,
            double turretPositionRots, double vxMps, double vyMps, double omegaRps) {
        // Turret pivot in field space
        double cosR = Math.cos(robotHeadingRad);
        double sinR = Math.sin(robotHeadingRad);
        double turretX = robotX + kTurretOffsetX_m * cosR - kTurretOffsetY_m * sinR;
        double turretY = robotY + kTurretOffsetX_m * sinR + kTurretOffsetY_m * cosR;

        // Vector from turret to target
        double dx = targetX - turretX;
        double dy = targetY - turretY;

        // Field yaw to target
        double fieldYawRad = Math.atan2(dy, dx);

        // Turret zero field direction
        double turretZeroRad = robotHeadingRad + kTurretAngleOffsetRad;

        // Direction in turret frame
        double dirRad = fieldYawRad - turretZeroRad;
        double dirRots = dirRad / (2 * Math.PI);

        // Normalize to turret range
        double angleRots = MathUtil.inputModulus(dirRots, kTurretMinRotsD, kTurretMaxRotsD);

        // Snapback
        double safeAngleRots = angleRots;
        if (turretPositionRots > 0 && angleRots + 1 <= kTurretMaxRotsD) {
            safeAngleRots += 1;
        } else if (turretPositionRots < 0 && angleRots - 1 >= kTurretMinRotsD) {
            safeAngleRots -= 1;
        }

        // Turret velocity FF
        double d2 = dx * dx + dy * dy;
        double turretFF = d2 > 0
            ? (dy * vxMps - dx * vyMps) / d2 - omegaRps
            : 0.0;

        return new double[]{safeAngleRots, turretFF};
    }

    /**
     * Full calcShot pipeline equivalent, raw doubles.
     * Returns [turretReferenceRots, hoodAngleDeg, shooterVelRadPerSec, turretFFRadPerSec].
     */
    private static double[] rawCalcShot(double robotX, double robotY, double robotHeadingRad,
            double turretPositionRots, double vxMps, double vyMps, double omegaRps,
            double targetX, double targetY, double targetZ) {

        double[] shotResult = rawIterativeMovingShot(
            robotX, robotY, robotHeadingRad, vxMps, vyMps, targetX, targetY, targetZ);
        double exitVelRadPerSec = shotResult[0];
        double hoodAngleRad = shotResult[1];
        double predTargetX = shotResult[2];
        double predTargetY = shotResult[3];

        double[] azResult = rawCalcAzimuth(
            predTargetX, predTargetY, targetZ,
            robotX, robotY, robotHeadingRad,
            turretPositionRots, vxMps, vyMps, omegaRps);

        double turretRefRots = azResult[0];
        double turretFFRadPerSec = azResult[1];
        double shooterVelRadPerSec = exitVelRadPerSec; // already in rad/s from LERP

        return new double[]{turretRefRots, hoodAngleRad, shooterVelRadPerSec, turretFFRadPerSec};
    }

    // ================================================================
    //  Benchmark
    // ================================================================

    private static final int WARMUP_ITERS = 50_000;
    private static final int BENCH_ITERS = 200_000;

    @Test
    void benchmarkComparison() {
        System.out.println("\n=== Shot Calc Performance Benchmark ===");
        System.out.println("Warmup iterations: " + WARMUP_ITERS);
        System.out.println("Benchmark iterations: " + BENCH_ITERS);
        System.out.println();

        // Extract raw values for approaches 2 & 3
        double robotX = MID_POSE.getX();
        double robotY = MID_POSE.getY();
        double robotHeadingRad = MID_POSE.getRotation().getRadians();
        double turretPosRots = 0.0;
        double vx = MOVING_SPEEDS.vxMetersPerSecond;
        double vy = MOVING_SPEEDS.vyMetersPerSecond;
        double omega = MOVING_SPEEDS.omegaRadiansPerSecond;
        double tX = HUB_TARGET.getX();
        double tY = HUB_TARGET.getY();
        double tZ = HUB_TARGET.getZ();

        // --- Warmup all three approaches ---
        for (int i = 0; i < WARMUP_ITERS; i++) {
            immutableCalcShot(MID_POSE, MOVING_SPEEDS, HUB_TARGET);
        }
        for (int i = 0; i < WARMUP_ITERS; i++) {
            mutableIterativeMovingShot(MID_POSE, MOVING_SPEEDS, HUB_TARGET);
        }
        for (int i = 0; i < WARMUP_ITERS; i++) {
            rawCalcShot(robotX, robotY, robotHeadingRad, turretPosRots, vx, vy, omega, tX, tY, tZ);
        }

        // --- Benchmark 1: Immutable Units ---
        long t0 = System.nanoTime();
        ShotData immutableResult = null;
        for (int i = 0; i < BENCH_ITERS; i++) {
            immutableResult = immutableCalcShot(MID_POSE, MOVING_SPEEDS, HUB_TARGET);
        }
        long immutableNs = System.nanoTime() - t0;

        // --- Benchmark 2: Mutable Units ---
        long t1 = System.nanoTime();
        ShotData mutableResult = null;
        for (int i = 0; i < BENCH_ITERS; i++) {
            mutableResult = mutableIterativeMovingShot(MID_POSE, MOVING_SPEEDS, HUB_TARGET);
        }
        long mutableNs = System.nanoTime() - t1;

        // --- Benchmark 3: Raw Doubles ---
        long t2 = System.nanoTime();
        double[] rawResult = null;
        for (int i = 0; i < BENCH_ITERS; i++) {
            rawResult = rawCalcShot(robotX, robotY, robotHeadingRad, turretPosRots, vx, vy, omega, tX, tY, tZ);
        }
        long rawNs = System.nanoTime() - t2;

        // --- Results ---
        double immutableUs = immutableNs / 1000.0 / BENCH_ITERS;
        double mutableUs = mutableNs / 1000.0 / BENCH_ITERS;
        double rawUs = rawNs / 1000.0 / BENCH_ITERS;

        System.out.println("┌─────────────────────┬──────────────┬───────────┐");
        System.out.println("│ Approach            │ us/call      │ vs raw    │");
        System.out.println("├─────────────────────┼──────────────┼───────────┤");
        System.out.printf("│ Immutable Units     │ %10.3f   │ %6.2fx   │%n", immutableUs, immutableUs / rawUs);
        System.out.printf("│ Mutable Units       │ %10.3f   │ %6.2fx   │%n", mutableUs, mutableUs / rawUs);
        System.out.printf("│ Raw Doubles         │ %10.3f   │ %6.2fx   │%n", rawUs, 1.0);
        System.out.println("└─────────────────────┴──────────────┴───────────┘");
        System.out.println();

        // Print estimated allocations per call
        System.out.println("Estimated heap allocations per call:");
        System.out.println("  Immutable Units: ~30-50 objects (Pose3d, Translation3d, Distance, Time, Angle, ShotData...)");
        System.out.println("  Mutable Units:   ~20-35 objects (still Pose3d/Translation3d, but reuses Distance/Time)");
        System.out.println("  Raw Doubles:      1 object (the double[] result array)");
        System.out.println();

        // Print at 25Hz (the ShooterCalc Notifier rate)
        System.out.println("At 25Hz ShooterCalc rate:");
        System.out.printf("  Immutable Units: %.1f us/cycle (%.1f%% of 40ms budget)%n",
            immutableUs, immutableUs / 40000.0 * 100);
        System.out.printf("  Mutable Units:   %.1f us/cycle (%.1f%% of 40ms budget)%n",
            mutableUs, mutableUs / 40000.0 * 100);
        System.out.printf("  Raw Doubles:     %.1f us/cycle (%.1f%% of 40ms budget)%n",
            rawUs, rawUs / 40000.0 * 100);
        System.out.println();

        // Verify raw doubles produce similar results to immutable
        System.out.println("Correctness check (immutable vs raw doubles):");
        System.out.printf("  Exit velocity: immutable=%.4f rad/s, raw=%.4f rad/s%n",
            immutableResult.exitVelocity(), rawResult[2]);
        System.out.printf("  Hood angle:    immutable=%.4f rad,   raw=%.4f rad%n",
            immutableResult.hoodAngle(), rawResult[1]);
        System.out.println();

        // The test always passes — it's a benchmark, not an assertion.
        // But let's sanity-check that raw doubles aren't wildly wrong.
        double exitVelDiff = Math.abs(immutableResult.exitVelocity() - rawResult[2]);
        double hoodDiff = Math.abs(immutableResult.hoodAngle() - rawResult[1]);
        System.out.printf("  Exit velocity delta: %.6f rad/s%n", exitVelDiff);
        System.out.printf("  Hood angle delta:    %.6f rad%n", hoodDiff);

        // These can differ somewhat because the raw LERP is a simplified reimplementation
        // of InterpolatingTreeMap, but should be in the same ballpark
        if (exitVelDiff < 5.0 && hoodDiff < 0.5) {
            System.out.println("  -> Results are in reasonable agreement.");
        } else {
            System.out.println("  -> WARNING: Results differ significantly. LERP reimplementation may need tuning.");
        }
    }
}
