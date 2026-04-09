package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.shooter.ShotCalculator.ShotData;
import frc.robot.subsystems.shooter.ShotCalculator.ShotDataLerp;
import frc.robot.subsystems.shooter.ShooterCalc.AzimuthCalcDetails;
import frc.robot.subsystems.shooter.ShooterCalc.ShotCalcOutputs;

import org.junit.jupiter.api.*;

/**
 * Characterization tests for the static shot calculation math.
 * These lock down current behavior before refactoring to raw-double equivalents.
 */
class ShotCalcTest {

    // ---- Shared test fixtures ----

    // Blue hub inner center (from FieldConstants, loaded at class init)
    private static Translation3d HUB_TARGET;

    // Representative robot poses at various distances from the blue hub
    private static Pose2d CLOSE_POSE;   // ~1.5m from hub
    private static Pose2d MID_POSE;     // ~3.9m from hub
    private static Pose2d FAR_POSE;     // ~5.7m from hub

    private static final ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds(0, 0, 0);
    private static final ChassisSpeeds TRANSLATING_SPEEDS = new ChassisSpeeds(2.0, 1.0, 0);
    private static final ChassisSpeeds ROTATING_SPEEDS = new ChassisSpeeds(0, 0, 1.0);
    private static final ChassisSpeeds COMBINED_SPEEDS = new ChassisSpeeds(1.5, -0.5, 0.5);

    @BeforeAll
    static void setup() {
        HAL.initialize(500, 0);

        // These depend on FieldConstants which loads AprilTag layout
        HUB_TARGET = frc.robot.FieldConstants.Hub.blueInnerCenterPoint;

        // Poses facing the hub from different distances (blue alliance side)
        // Hub is roughly at x~4.6, y~4.0 (field center-ish)
        double hubX = HUB_TARGET.getX();
        double hubY = HUB_TARGET.getY();

        CLOSE_POSE = new Pose2d(hubX - 1.5, hubY, Rotation2d.kZero);
        MID_POSE = new Pose2d(hubX - 3.9, hubY + 1.0, Rotation2d.fromDegrees(15));
        FAR_POSE = new Pose2d(hubX - 5.7, hubY - 1.5, Rotation2d.fromDegrees(-20));
    }

    // ================================================================
    //  ShotCalculator: simple unit conversion functions
    // ================================================================

    @Nested
    class LinearAngularConversions {
        @Test
        void linearToAngular_basicConversion() {
            // v = 10 m/s, r = 0.5 m -> omega = v/r = 20 rad/s
            var result = ShotCalculator.linearToAngularVelocity(
                MetersPerSecond.of(10.0), Meters.of(0.5));
            assertEquals(20.0, result.in(RadiansPerSecond), 1e-9);
        }

        @Test
        void angularToLinear_basicConversion() {
            // omega = 20 rad/s, r = 0.5 m -> v = omega*r = 10 m/s
            var result = ShotCalculator.angularToLinearVelocity(
                RadiansPerSecond.of(20.0), Meters.of(0.5));
            assertEquals(10.0, result.in(MetersPerSecond), 1e-9);
        }

        @Test
        void roundTrip_linearToAngularAndBack() {
            LinearVelocity original = MetersPerSecond.of(7.3);
            Distance radius = Inches.of(1.5); // flywheel radius
            AngularVelocity angular = ShotCalculator.linearToAngularVelocity(original, radius);
            LinearVelocity recovered = ShotCalculator.angularToLinearVelocity(angular, radius);
            assertEquals(original.in(MetersPerSecond), recovered.in(MetersPerSecond), 1e-9);
        }

        @Test
        void zeroVelocity() {
            var result = ShotCalculator.linearToAngularVelocity(
                MetersPerSecond.of(0), Meters.of(0.5));
            assertEquals(0.0, result.in(RadiansPerSecond), 1e-9);
        }
    }

    // ================================================================
    //  ShotCalculator.getDistanceToTarget
    // ================================================================

    @Nested
    class GetDistanceToTarget {
        @Test
        void closePose_returnsPositiveDistance() {
            Distance d = ShotCalculator.getDistanceToTarget(CLOSE_POSE, HUB_TARGET);
            assertTrue(d.in(Meters) > 0, "Distance should be positive");
            // Close pose is ~1.5m from hub, but turret transform offsets it
            assertTrue(d.in(Meters) < 3.0, "Close pose should be within 3m");
        }

        @Test
        void midPose_returnsReasonableDistance() {
            Distance d = ShotCalculator.getDistanceToTarget(MID_POSE, HUB_TARGET);
            assertTrue(d.in(Meters) > 2.0);
            assertTrue(d.in(Meters) < 6.0);
        }

        @Test
        void farPose_returnsLargerDistance() {
            Distance dClose = ShotCalculator.getDistanceToTarget(CLOSE_POSE, HUB_TARGET);
            Distance dFar = ShotCalculator.getDistanceToTarget(FAR_POSE, HUB_TARGET);
            assertTrue(dFar.in(Meters) > dClose.in(Meters),
                "Far pose should be further than close pose");
        }

        @Test
        void samePosition_returnsSmallDistance() {
            // Robot right at the hub X,Y — distance should just be the turret transform offset
            Pose2d atHub = new Pose2d(HUB_TARGET.getX(), HUB_TARGET.getY(), Rotation2d.kZero);
            Distance d = ShotCalculator.getDistanceToTarget(atHub, HUB_TARGET);
            // Should be small but nonzero due to turret offset
            assertTrue(d.in(Meters) < 1.0);
            assertTrue(d.in(Meters) >= 0);
        }

        @Test
        void isConsistent_withDifferentHeadings() {
            // Robot heading shouldn't drastically change the 2D distance
            // (turret transform rotates with robot, so there IS some effect)
            Pose2d heading0 = new Pose2d(2.0, 4.0, Rotation2d.kZero);
            Pose2d heading90 = new Pose2d(2.0, 4.0, Rotation2d.kCCW_90deg);
            Distance d0 = ShotCalculator.getDistanceToTarget(heading0, HUB_TARGET);
            Distance d90 = ShotCalculator.getDistanceToTarget(heading90, HUB_TARGET);
            // Both should be in a similar ballpark (within turret offset range ~0.17m)
            assertEquals(d0.in(Meters), d90.in(Meters), 0.5);
        }
    }

    // ================================================================
    //  ShotCalculator.calculateTimeOfFlight
    // ================================================================

    @Nested
    class CalculateTimeOfFlight {
        @Test
        void basicTimeOfFlight() {
            // v = 10 m/s at 45 deg hood -> horizontal = 10*cos(45deg) ≈ 7.07 m/s
            // distance = 3m -> tof = 3/7.07 ≈ 0.424 s
            // Note: hood angle 0 = horizontal in this function (pi/2 - hoodAngle)
            // Actually from code: angle = PI/2 - hoodAngle.in(Radians)
            // So hoodAngle of 45 deg -> effective angle = 45 deg -> cos(45deg) component
            Time tof = ShotCalculator.calculateTimeOfFlight(
                MetersPerSecond.of(10.0),
                Degrees.of(45),
                Meters.of(3.0));
            double tofSec = tof.in(Seconds);
            assertFalse(Double.isNaN(tofSec), "TOF should not be NaN");
            assertTrue(tofSec > 0, "TOF should be positive");
        }

        @Test
        void longerDistance_longerTOF() {
            Time tofShort = ShotCalculator.calculateTimeOfFlight(
                MetersPerSecond.of(10.0), Degrees.of(30), Meters.of(2.0));
            Time tofLong = ShotCalculator.calculateTimeOfFlight(
                MetersPerSecond.of(10.0), Degrees.of(30), Meters.of(5.0));
            assertTrue(tofLong.in(Seconds) > tofShort.in(Seconds));
        }

        @Test
        void fasterVelocity_shorterTOF() {
            Time tofSlow = ShotCalculator.calculateTimeOfFlight(
                MetersPerSecond.of(5.0), Degrees.of(30), Meters.of(3.0));
            Time tofFast = ShotCalculator.calculateTimeOfFlight(
                MetersPerSecond.of(15.0), Degrees.of(30), Meters.of(3.0));
            assertTrue(tofFast.in(Seconds) < tofSlow.in(Seconds));
        }
    }

    // ================================================================
    //  ShotCalculator.predictTargetPos
    // ================================================================

    @Nested
    class PredictTargetPos {
        @Test
        void stationaryRobot_targetUnchanged() {
            Translation3d target = new Translation3d(5, 4, 1.5);
            Translation3d predicted = ShotCalculator.predictTargetPos(
                target, ZERO_SPEEDS, Seconds.of(1.0));
            assertEquals(target.getX(), predicted.getX(), 1e-9);
            assertEquals(target.getY(), predicted.getY(), 1e-9);
            assertEquals(target.getZ(), predicted.getZ(), 1e-9);
        }

        @Test
        void movingRobot_targetShiftsOppositeToVelocity() {
            Translation3d target = new Translation3d(5, 4, 1.5);
            ChassisSpeeds speeds = new ChassisSpeeds(2.0, 0, 0); // moving +x at 2 m/s
            Translation3d predicted = ShotCalculator.predictTargetPos(
                target, speeds, Seconds.of(1.0));

            // predicted.x = target.x - vx * tof = 5 - 2*1 = 3
            assertEquals(3.0, predicted.getX(), 1e-9);
            assertEquals(4.0, predicted.getY(), 1e-9);
            assertEquals(1.5, predicted.getZ(), 1e-9); // Z unchanged
        }

        @Test
        void zeroTOF_targetUnchanged() {
            Translation3d target = new Translation3d(5, 4, 1.5);
            Translation3d predicted = ShotCalculator.predictTargetPos(
                target, TRANSLATING_SPEEDS, Seconds.of(0));
            assertEquals(target.getX(), predicted.getX(), 1e-9);
            assertEquals(target.getY(), predicted.getY(), 1e-9);
        }

        @Test
        void movingInY_shiftsY() {
            Translation3d target = new Translation3d(5, 4, 1.5);
            ChassisSpeeds speeds = new ChassisSpeeds(0, 3.0, 0); // moving +y at 3 m/s
            Translation3d predicted = ShotCalculator.predictTargetPos(
                target, speeds, Seconds.of(0.5));
            assertEquals(5.0, predicted.getX(), 1e-9);
            assertEquals(4.0 - 3.0 * 0.5, predicted.getY(), 1e-9); // 4 - 1.5 = 2.5
        }
    }

    // ================================================================
    //  ShotCalculator.iterativeMovingShotFromInterpolationMap
    // ================================================================

    @Nested
    class IterativeMovingShotFromInterpolationMap {
        @Test
        void stationaryClose_returnsValidShot() {
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                CLOSE_POSE, ZERO_SPEEDS, HUB_TARGET, 3);
            assertShotDataValid(shot);
        }

        @Test
        void stationaryMid_returnsValidShot() {
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                MID_POSE, ZERO_SPEEDS, HUB_TARGET, 3);
            assertShotDataValid(shot);
        }

        @Test
        void stationaryFar_returnsValidShot() {
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                FAR_POSE, ZERO_SPEEDS, HUB_TARGET, 3);
            assertShotDataValid(shot);
        }

        @Test
        void movingRobot_returnsValidShot() {
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                MID_POSE, TRANSLATING_SPEEDS, HUB_TARGET, 3);
            assertShotDataValid(shot);
        }

        @Test
        void rotatingRobot_returnsValidShot() {
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                MID_POSE, ROTATING_SPEEDS, HUB_TARGET, 3);
            assertShotDataValid(shot);
        }

        @Test
        void stationaryShot_targetMatchesInput() {
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                MID_POSE, ZERO_SPEEDS, HUB_TARGET, 3);
            // With zero speeds, predicted target should equal actual target
            assertEquals(HUB_TARGET.getX(), shot.getTarget().getX(), 1e-6);
            assertEquals(HUB_TARGET.getY(), shot.getTarget().getY(), 1e-6);
            assertEquals(HUB_TARGET.getZ(), shot.getTarget().getZ(), 1e-6);
        }

        @Test
        void movingShot_targetDiffersFromStatic() {
            ShotDataLerp staticShot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                MID_POSE, ZERO_SPEEDS, HUB_TARGET, 3);
            ShotDataLerp movingShot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                MID_POSE, TRANSLATING_SPEEDS, HUB_TARGET, 3);
            // Moving shot should shift the predicted target
            assertNotEquals(staticShot.getTarget().getX(), movingShot.getTarget().getX(), 1e-6,
                "Moving shot predicted target X should differ from static");
        }

        @Test
        void moreIterations_convergesToSameResult() {
            // The iterative solver exhibits damped oscillation (alternating over/undershoot)
            // with a ~1/3 contraction ratio per step. At vx=2.0, vy=1.0 m/s it needs ~7
            // iterations to fully converge. Production uses 5 (ShooterCalc.calcShot),
            // so we verify that 5 iterations lands close to the fully-converged answer.
            ShotDataLerp shot5 = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                MID_POSE, TRANSLATING_SPEEDS, HUB_TARGET, 5);
            ShotDataLerp shot10 = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                MID_POSE, TRANSLATING_SPEEDS, HUB_TARGET, 10);
            assertEquals(shot5.exitVelocity(), shot10.exitVelocity(), 0.5,
                "5 and 10 iterations should converge on exit velocity");
            assertEquals(shot5.hoodAngle(), shot10.hoodAngle(), 0.1,
                "5 and 10 iterations should converge on hood angle");
        }
    }

    // ================================================================
    //  ShotCalculator.calculateShotFromFunnelClearance
    // ================================================================

    @Nested
    class CalculateShotFromFunnelClearance {
        @Test
        void stationaryClose_returnsValidShot() {
            ShotData shot = ShotCalculator.calculateShotFromFunnelClearance(
                CLOSE_POSE, HUB_TARGET, HUB_TARGET);
            assertShotDataValid(shot);
        }

        @Test
        void stationaryMid_returnsValidShot() {
            ShotData shot = ShotCalculator.calculateShotFromFunnelClearance(
                MID_POSE, HUB_TARGET, HUB_TARGET);
            assertShotDataValid(shot);
        }

        @Test
        void stationaryFar_returnsValidShot() {
            ShotData shot = ShotCalculator.calculateShotFromFunnelClearance(
                FAR_POSE, HUB_TARGET, HUB_TARGET);
            assertShotDataValid(shot);
        }
    }

    // ================================================================
    //  LERP edge-distance SOTM sensitivity
    // ================================================================

    @Nested
    class LerpEdgePredictedDistanceShift {
        // Poses placed at exact distances from the hub along the -X axis (facing hub).
        // The turret offset shifts the effective distance slightly, but the pose X offset
        // is the dominant contributor.

        /** Build a pose at roughly `dist` meters from the hub, facing it. */
        private Pose2d poseAtDistance(double dist) {
            return new Pose2d(HUB_TARGET.getX() - dist, HUB_TARGET.getY(), Rotation2d.kZero);
        }

        /** Euclidean XY shift between the predicted target and the actual target. */
        private double predShift(ShotDataLerp shot) {
            double dx = shot.getTarget().getX() - HUB_TARGET.getX();
            double dy = shot.getTarget().getY() - HUB_TARGET.getY();
            return Math.sqrt(dx * dx + dy * dy);
        }

        // ---- Near the minimum LERP key (1.168 m) ----

        @Test
        void nearMinDistance_lowSpeed_smallShift() {
            Pose2d pose = poseAtDistance(1.3);
            ChassisSpeeds slow = new ChassisSpeeds(0.5, 0, 0);
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                pose, slow, HUB_TARGET, 5);
            assertShotDataValid(shot);
            double shift = predShift(shot);
            assertTrue(shift < 1.0,
                "At 1.3m with 0.5 m/s the predicted target should shift < 1m, got " + shift);
        }

        @Test
        void nearMinDistance_highSpeed_largerShift() {
            Pose2d pose = poseAtDistance(1.3);
            ChassisSpeeds fast = new ChassisSpeeds(3.0, 1.5, 0);
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                pose, fast, HUB_TARGET, 5);
            assertShotDataValid(shot);
            double shift = predShift(shot);
            assertTrue(shift > 0.1,
                "At 1.3m with 3.0 m/s the predicted target should shift noticeably, got " + shift);
        }

        @Test
        void nearMinDistance_speedSweep_shiftMonotonic() {
            Pose2d pose = poseAtDistance(1.3);
            double prevShift = 0;
            for (double vx = 0.5; vx <= 4.0; vx += 0.5) {
                ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                    pose, new ChassisSpeeds(vx, 0, 0), HUB_TARGET, 5);
                assertShotDataValid(shot);
                double shift = predShift(shot);
                assertTrue(shift >= prevShift - 0.01,
                    String.format("At 1.3m, shift should grow with speed: vx=%.1f shift=%.4f prev=%.4f",
                        vx, shift, prevShift));
                prevShift = shift;
            }
        }

        // ---- Near the maximum LERP key (7.565 m) ----

        @Test
        void nearMaxDistance_lowSpeed_smallShift() {
            Pose2d pose = poseAtDistance(7.0);
            ChassisSpeeds slow = new ChassisSpeeds(0.5, 0, 0);
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                pose, slow, HUB_TARGET, 5);
            assertShotDataValid(shot);
            double shift = predShift(shot);
            assertTrue(shift < 2.0,
                "At 7.0m with 0.5 m/s the predicted target should shift < 2m, got " + shift);
        }

        @Test
        void nearMaxDistance_highSpeed_largerShift() {
            Pose2d pose = poseAtDistance(7.0);
            ChassisSpeeds fast = new ChassisSpeeds(3.0, 1.5, 0);
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                pose, fast, HUB_TARGET, 5);
            assertShotDataValid(shot);
            double shift = predShift(shot);
            assertTrue(shift > 0.3,
                "At 7.0m with 3.0 m/s the predicted target should shift noticeably, got " + shift);
        }

        @Test
        void nearMaxDistance_speedSweep_shiftMonotonic() {
            Pose2d pose = poseAtDistance(7.0);
            double prevShift = 0;
            for (double vx = 0.5; vx <= 4.0; vx += 0.5) {
                ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                    pose, new ChassisSpeeds(vx, 0, 0), HUB_TARGET, 5);
                assertShotDataValid(shot);
                double shift = predShift(shot);
                assertTrue(shift >= prevShift - 0.01,
                    String.format("At 7.0m, shift should grow with speed: vx=%.1f shift=%.4f prev=%.4f",
                        vx, shift, prevShift));
                prevShift = shift;
            }
        }

        // ---- Beyond the LERP table (extrapolation) ----

        @Test
        void beyondMaxDistance_returnsValidShot() {
            Pose2d pose = poseAtDistance(9.0);
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                pose, TRANSLATING_SPEEDS, HUB_TARGET, 5);
            assertShotDataValid(shot);
        }

        @Test
        void belowMinDistance_returnsValidShot() {
            Pose2d pose = poseAtDistance(0.8);
            ShotDataLerp shot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                pose, TRANSLATING_SPEEDS, HUB_TARGET, 5);
            assertShotDataValid(shot);
        }

        // ---- Shift comparison: close vs far at same speed ----

        @Test
        void farShot_largerShiftThanClose_atSameSpeed() {
            ChassisSpeeds speed = new ChassisSpeeds(2.0, 0, 0);
            ShotDataLerp closeShot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                poseAtDistance(1.5), speed, HUB_TARGET, 5);
            ShotDataLerp farShot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                poseAtDistance(6.0), speed, HUB_TARGET, 5);
            double closeShift = predShift(closeShot);
            double farShift = predShift(farShot);
            assertTrue(farShift > closeShift,
                String.format("Far shot shift (%.4f) should exceed close shot shift (%.4f) at same speed",
                    farShift, closeShift));
        }

        // ---- Lateral velocity ----

        @Test
        void lateralVelocity_shiftsYComponent() {
            Pose2d pose = poseAtDistance(4.0);
            ShotDataLerp shotY = ShotCalculator.iterativeMovingShotFromInterpolationMap(
                pose, new ChassisSpeeds(0, 2.0, 0), HUB_TARGET, 5);
            assertShotDataValid(shotY);
            double dy = Math.abs(shotY.getTarget().getY() - HUB_TARGET.getY());
            assertTrue(dy > 0.1,
                "Lateral velocity should shift predicted target Y, got dy=" + dy);
        }
    }

    // ================================================================
    //  ShotCalculator.iterativeMovingShotFromFunnelClearance
    // ================================================================

    @Nested
    class IterativeMovingShotFromFunnelClearance {
        @Test
        void stationaryMid_returnsValidShot() {
            ShotData shot = ShotCalculator.iterativeMovingShotFromFunnelClearance(
                MID_POSE, ZERO_SPEEDS, HUB_TARGET, 3);
            assertShotDataValid(shot);
        }

        @Test
        void movingRobot_returnsValidShot() {
            ShotData shot = ShotCalculator.iterativeMovingShotFromFunnelClearance(
                MID_POSE, TRANSLATING_SPEEDS, HUB_TARGET, 3);
            assertShotDataValid(shot);
        }
    }

    // ================================================================
    //  ShooterCalc.calcAzimuth (static method)
    // ================================================================

    @Nested
    class CalcAzimuth {
        @Test
        void turretAtZero_closeTarget_returnsValidAzimuth() {
            AzimuthCalcDetails details = ShooterCalc.calcAzimuth(
                HUB_TARGET, CLOSE_POSE, 0.0, ZERO_SPEEDS);
            assertAzimuthValid(details);
        }

        @Test
        void turretAtZero_midTarget_returnsValidAzimuth() {
            AzimuthCalcDetails details = ShooterCalc.calcAzimuth(
                HUB_TARGET, MID_POSE, 0.0, ZERO_SPEEDS);
            assertAzimuthValid(details);
        }

        @Test
        void turretAtPositiveQuarter_returnsValidAzimuth() {
            AzimuthCalcDetails details = ShooterCalc.calcAzimuth(
                HUB_TARGET, MID_POSE, 0.25, ZERO_SPEEDS);
            assertAzimuthValid(details);
        }

        @Test
        void turretAtNegativeQuarter_returnsValidAzimuth() {
            AzimuthCalcDetails details = ShooterCalc.calcAzimuth(
                HUB_TARGET, MID_POSE, -0.25, ZERO_SPEEDS);
            assertAzimuthValid(details);
        }

        @Test
        void movingRobot_producesVelocityFF() {
            AzimuthCalcDetails stationary = ShooterCalc.calcAzimuth(
                HUB_TARGET, MID_POSE, 0.0, ZERO_SPEEDS);
            AzimuthCalcDetails moving = ShooterCalc.calcAzimuth(
                HUB_TARGET, MID_POSE, 0.0, TRANSLATING_SPEEDS);
            // Moving robot should produce nonzero velocity feedforward
            assertNotEquals(0.0, moving.turretVelocityFF(), 1e-6,
                "Moving robot should produce nonzero turret velocity FF");
        }

        @Test
        void stationaryRobot_zeroVelocityFF() {
            AzimuthCalcDetails details = ShooterCalc.calcAzimuth(
                HUB_TARGET, MID_POSE, 0.0, ZERO_SPEEDS);
            assertEquals(0.0, details.turretVelocityFF(), 1e-6,
                "Stationary robot should have zero velocity FF (omega=0, v=0)");
        }

        @Test
        void turretReference_withinPhysicalLimits() {
            // Test across many poses and turret positions
            Pose2d[] poses = {CLOSE_POSE, MID_POSE, FAR_POSE};
            double[] turretRots = {-0.5, -0.25, 0, 0.25, 0.5};
            for (Pose2d pose : poses) {
                for (double rot : turretRots) {
                    AzimuthCalcDetails d = ShooterCalc.calcAzimuth(
                        HUB_TARGET, pose, rot, ZERO_SPEEDS);
                    double refRots = d.turretReferenceRots();
                    assertTrue(refRots >= -0.76 && refRots <= 0.76,
                        String.format("Turret reference %.4f out of [-0.75, 0.75] for pose=%s, turret=%.2f",
                            refRots, pose, rot));
                }
            }
        }
    }

    // ================================================================
    //  ShooterCalc.calcShot (static method, full pipeline)
    // ================================================================

    @Nested
    class CalcShot {
        @Test
        void staticShot_close_returnsValidOutputs() {
            ShotCalcOutputs out = ShooterCalc.calcShot(
                CLOSE_POSE, true, HUB_TARGET, 0.0, ZERO_SPEEDS);
            assertShotCalcOutputsValid(out);
        }

        @Test
        void staticShot_mid_returnsValidOutputs() {
            ShotCalcOutputs out = ShooterCalc.calcShot(
                MID_POSE, true, HUB_TARGET, 0.0, ZERO_SPEEDS);
            assertShotCalcOutputsValid(out);
        }

        @Test
        void staticShot_far_returnsValidOutputs() {
            ShotCalcOutputs out = ShooterCalc.calcShot(
                FAR_POSE, true, HUB_TARGET, 0.0, ZERO_SPEEDS);
            assertShotCalcOutputsValid(out);
        }

        @Test
        void dynamicShot_translating_returnsValidOutputs() {
            ShotCalcOutputs out = ShooterCalc.calcShot(
                MID_POSE, false, HUB_TARGET, 0.0, TRANSLATING_SPEEDS);
            assertShotCalcOutputsValid(out);
        }

        @Test
        void dynamicShot_rotating_returnsValidOutputs() {
            ShotCalcOutputs out = ShooterCalc.calcShot(
                MID_POSE, false, HUB_TARGET, 0.0, ROTATING_SPEEDS);
            assertShotCalcOutputsValid(out);
        }

        @Test
        void dynamicShot_combined_returnsValidOutputs() {
            ShotCalcOutputs out = ShooterCalc.calcShot(
                MID_POSE, false, HUB_TARGET, 0.1, COMBINED_SPEEDS);
            assertShotCalcOutputsValid(out);
        }

        @Test
        void staticVsDynamic_sameWhenStationary() {
            ShotCalcOutputs staticOut = ShooterCalc.calcShot(
                MID_POSE, true, HUB_TARGET, 0.0, ZERO_SPEEDS);
            ShotCalcOutputs dynamicOut = ShooterCalc.calcShot(
                MID_POSE, false, HUB_TARGET, 0.0, ZERO_SPEEDS);
            // With zero chassis speeds, static and dynamic should produce identical results
            assertEquals(
                staticOut.shooterReferenceRps(),
                dynamicOut.shooterReferenceRps(),
                1e-6, "Zero-speed dynamic should match static shot");
            assertEquals(
                staticOut.hoodReferenceRots(),
                dynamicOut.hoodReferenceRots(),
                1e-6);
            assertEquals(
                staticOut.turretReferenceRots(),
                dynamicOut.turretReferenceRots(),
                1e-6);
        }

        @Test
        void farShot_higherVelocityThanClose() {
            ShotCalcOutputs closeOut = ShooterCalc.calcShot(
                CLOSE_POSE, true, HUB_TARGET, 0.0, ZERO_SPEEDS);
            ShotCalcOutputs farOut = ShooterCalc.calcShot(
                FAR_POSE, true, HUB_TARGET, 0.0, ZERO_SPEEDS);
            assertTrue(
                farOut.shooterReferenceRps() >
                closeOut.shooterReferenceRps(),
                "Far shot should require higher flywheel velocity");
        }
    }

    // ================================================================
    //  ShotData record tests
    // ================================================================

    @Nested
    class ShotDataTests {
        @Test
        void interpolate_midpoint() {
            ShotData a = new ShotData(10.0, 1.0);
            ShotData b = new ShotData(20.0, 2.0);
            ShotData mid = ShotData.interpolate(a, b, 0.5);
            assertEquals(15.0, mid.exitVelocity(), 1e-9);
            assertEquals(1.5, mid.hoodAngle(), 1e-9);
        }

        @Test
        void interpolate_atStart() {
            ShotData a = new ShotData(10.0, 1.0);
            ShotData b = new ShotData(20.0, 2.0);
            ShotData result = ShotData.interpolate(a, b, 0.0);
            assertEquals(10.0, result.exitVelocity(), 1e-9);
            assertEquals(1.0, result.hoodAngle(), 1e-9);
        }

        @Test
        void interpolate_atEnd() {
            ShotData a = new ShotData(10.0, 1.0);
            ShotData b = new ShotData(20.0, 2.0);
            ShotData result = ShotData.interpolate(a, b, 1.0);
            assertEquals(20.0, result.exitVelocity(), 1e-9);
            assertEquals(2.0, result.hoodAngle(), 1e-9);
        }

        @Test
        void minus_computesDifference() {
            ShotData a = new ShotData(15.0, 1.5);
            ShotData b = new ShotData(10.0, 1.0);
            ShotData diff = a.minus(b);
            assertEquals(10.0 - 15.0, diff.exitVelocity(), 1e-9);
            assertEquals(1.0 - 1.5, diff.hoodAngle(), 1e-9);
        }

        @Test
        void getExitVelocity_convertsFromRadPerSec() {
            ShotData data = new ShotData(
                RotationsPerSecond.of(50), Degrees.of(45));
            LinearVelocity vel = data.getExitVelocity();
            assertFalse(Double.isNaN(vel.in(MetersPerSecond)));
            assertTrue(vel.in(MetersPerSecond) > 0);
        }

        @Test
        void getHoodAngle_convertsFromRadians() {
            ShotData data = new ShotData(
                RotationsPerSecond.of(50), Degrees.of(45));
            Angle angle = data.getHoodAngle();
            assertEquals(45.0, angle.in(Degrees), 0.01);
        }
    }

    // ================================================================
    //  Assertion helpers
    // ================================================================

    private static void assertShotDataValid(ShotData shot) {
        assertNotNull(shot, "ShotData should not be null");
        assertFalse(Double.isNaN(shot.exitVelocity()),
            "Exit velocity should not be NaN");
        assertFalse(Double.isNaN(shot.hoodAngle()),
            "Hood angle should not be NaN");
        assertFalse(Double.isInfinite(shot.exitVelocity()),
            "Exit velocity should not be infinite");
        assertFalse(Double.isInfinite(shot.hoodAngle()),
            "Hood angle should not be infinite");
        assertNotNull(shot.getTarget(), "Shot target should not be null");
    }

    private static void assertShotDataValid(ShotDataLerp shot) {
        assertNotNull(shot, "ShotData should not be null");
        assertFalse(Double.isNaN(shot.exitVelocity()),
            "Exit velocity should not be NaN");
        assertFalse(Double.isNaN(shot.hoodAngle()),
            "Hood angle should not be NaN");
        assertFalse(Double.isInfinite(shot.exitVelocity()),
            "Exit velocity should not be infinite");
        assertFalse(Double.isInfinite(shot.hoodAngle()),
            "Hood angle should not be infinite");
        assertNotNull(shot.getTarget(), "Shot target should not be null");
        // TODO: add tofSec
    }

    private static void assertAzimuthValid(AzimuthCalcDetails details) {
        assertNotNull(details, "AzimuthCalcDetails should not be null");
        assertFalse(Double.isNaN(details.turretReferenceRots()),
            "Turret reference should not be NaN");
        assertFalse(Double.isNaN(details.rawDesiredRotations()),
            "Raw desired rotations should not be NaN");
        assertFalse(Double.isNaN(details.turretVelocityFF()),
            "Turret velocity FF should not be NaN");
    }

    private static void assertShotCalcOutputsValid(ShotCalcOutputs out) {
        assertNotNull(out, "ShotCalcOutputs should not be null");
        assertAzimuthValid(out.turretCalcDetails());
        assertNotNull(out.shotData(), "ShotData should not be null");
        assertShotDataValid(out.shotData());
        assertFalse(Double.isNaN(out.turretReferenceRots()),
            "Turret reference should not be NaN");
        assertFalse(Double.isNaN(out.hoodReferenceRots()),
            "Hood reference should not be NaN");
        assertFalse(Double.isNaN(out.shooterReferenceRps()),
            "Shooter reference should not be NaN");
        assertTrue(out.shooterReferenceRps() >= 0,
            "Shooter velocity should be non-negative");
    }
}
