
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShooterK.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Tracer;
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
    // private static final Tracer m_iterativeTracer = new Tracer();

    //see 5000's code (circa 2/16/2026 9:11 PM EST)
    public static final InterpolatingTreeMap<Double, ShotData> m_shotMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShotData::interpolate);

    public static final InterpolatingDoubleTreeMap m_timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private static final double minDistance;
    private static final double maxDistance;

    //LERP MADE ON 3/15/2025
    static {
        //TODO: find the actual minDistance and maxDistance for shooting
        minDistance = 0.94;
        maxDistance = 5.8;

        //mid grid
        m_shotMap.put(3.903, new ShotData(RotationsPerSecond.of(73.18), Degrees.of(327.75)));
        m_timeOfFlightMap.put(3.903, 1.35);

        //backright grid - THIS ONE IS INCONSISTENT UNTIL SHOOTER V3
        m_shotMap.put(5.657, new ShotData(RotationsPerSecond.of(95.00), Degrees.of(374.20)));
        m_timeOfFlightMap.put(5.657, 1.56);

        //frontright grid
        m_shotMap.put(3.763, new ShotData(RotationsPerSecond.of(77), Degrees.of(300)));
        m_timeOfFlightMap.put(3.763, 1.43);

        //frontleft grid
        m_shotMap.put(1.476, new ShotData(RotationsPerSecond.of(57.72), Degrees.of(150.77)));
        m_timeOfFlightMap.put(1.476, 1.125);

        //backleft grid
        m_shotMap.put(3.854, new ShotData(RotationsPerSecond.of(79), Degrees.of(345)));
        m_timeOfFlightMap.put(3.854, 1.38);
    }

    /**
     * Gets the Distance from current robot position to desired target.
     * Intended for shot calculation
     * @param robot current robot Pose
     * @param target desired target pose
     * @return the distance between the Robot and the Target
     */
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        Pose3d turretPose = new Pose3d(robot).transformBy(kTurretTransform);
        Distance dist = Meters.of(turretPose.getTranslation().toTranslation2d().getDistance(target.toTranslation2d()));
        log_distToTargetMeters.accept(dist.magnitude());
        return dist;
    }

    // see https://www.desmos.com/geometry/l4edywkmha
    public static Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity,
            Translation3d target) {
        double vel = velocity.in(InchesPerSecond);
        double x_dist = getDistanceToTarget(robot, target).in(Inches);
        double y_dist = target.getMeasureZ().minus(kTurretTransform.getMeasureZ()).in(Inches);

        double angle = Math.atan(((vel * vel) + Math.sqrt(
                Math.pow(vel, 4) - kGravity * (kGravity * x_dist * x_dist + 2 * y_dist * vel * vel)))
                / (kGravity  * x_dist));

        return Radians.of(angle);
    }

    //calculate how long it will take for a projectile to travel a certain amount of distance given its initial velocity and angle
    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle,
            Distance distance) {
        double vel = exitVelocity.in(MetersPerSecond);
        double angle = Math.PI / 2 - hoodAngle.in(Radians);
        double dist = distance.in(Meters);

        log_timeOfFlight.accept(Seconds.of(dist / (vel * Math.cos(angle))).magnitude());

        return Seconds.of(dist / (vel * Math.cos(angle)));
    }

    public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
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
        double x_dist = getDistanceToTarget(robot, predictedTarget).in(Inches);
        double y_dist = predictedTarget.getMeasureZ().minus(kTurretTransform.getMeasureZ())
                .in(Inches);
        double g = 386;
        double r = FieldConstants.Hub.funnelRadius.in(Inches) * x_dist
                / getDistanceToTarget(robot, actualTarget).in(Inches);
        double h = FieldConstants.Hub.funnelHeight.plus(kInchesAboveFunnel).in(Inches);
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

        return new ShotData(linearToAngularVelocity(InchesPerSecond.of(v0), kFlywheelRadius),
                Radians.of(Math.PI / 2 - theta), predictedTarget);

    }

    // use an iterative lookahead approach to determine shot parameters for a moving robot
    public static ShotData iterativeMovingShotFromFunnelClearance(Pose2d robot,
            ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
        // m_iterativeTracer.clearEpochs();

        // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
        ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
        // m_iterativeTracer.addEpoch("initialShot");

        Distance distance = getDistanceToTarget(robot, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(),
                distance);
        // m_iterativeTracer.addEpoch("initialTOF");

        Translation3d predictedTarget = target;

        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        for (int i = 0; i < iterations; i++) {
            // ShotData prevShot = shot;
            // Distance prevDistance = distance;
            // Time prevTOF = timeOfFlight;
            // Translation3d prevPredTarget = predictedTarget;

            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(),
                    getDistanceToTarget(robot, predictedTarget));
            // m_iterativeTracer.addEpoch("iteration" + i);

            // ShotData shotDataDiff = shot.minus(prevShot);
            // Distance distanceDiff = prevDistance.minus(distance);
            // Time timeOfFlightDiff = prevTOF.minus(timeOfFlight);

            // if (shotDataDiff.hoodAngle() < .05 && shotDataDiff.exitVelocity() < .05 && prevShot.target().getDistance(shot.getTarget()) < .05
            //         && distanceDiff.magnitude() < .05 && timeOfFlightDiff.magnitude() < .005 && prevPredTarget.getDistance(predictedTarget) < .05) {
            //     break;
            // }
        }

        // m_iterativeTracer.printEpochs();
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

        double distance = getDistanceToTarget(robot, target).in(Meters);
        ShotData shot = m_shotMap.get(distance);
        shot = new ShotData(shot.exitVelocity(), shot.hoodAngle(), target);
        Time timeOfFlight = Seconds.of(m_timeOfFlightMap.get(distance));
        Translation3d predictedTarget = target;

        //meant to iterate the process, and converge on one specific value.
        //gets a better ToF estimation & updates predictedTarget accordingly!
        for (int i = 0; i < iterations; i++) {
            ShotData prevShot = shot;
            Time prevTOF = timeOfFlight;
            Translation3d prevPredTarget = predictedTarget;

            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            distance = getDistanceToTarget(robot, predictedTarget).in(Meters);
            shot = m_shotMap.get(distance);
            shot = new ShotData(shot.exitVelocity, shot.hoodAngle, predictedTarget);
            timeOfFlight = Seconds.of(m_timeOfFlightMap.get(distance));

            ShotData shotDataDiff = shot.minus(prevShot);
            Time timeOfFlightDiff = prevTOF.minus(timeOfFlight);

            if (shotDataDiff.hoodAngle() < .05 && shotDataDiff.exitVelocity() < .05
                    && prevShot.target().getDistance(shot.getTarget()) < .05
                    && timeOfFlightDiff.magnitude() < .005
                    && prevPredTarget.getDistance(predictedTarget) < .05) {
                break;
            }
        }

        return shot;
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
