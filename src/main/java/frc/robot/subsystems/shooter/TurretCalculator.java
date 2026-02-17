package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShooterK.kDistanceAboveFunnel;
import static frc.robot.Constants.ShooterK.kFlywheelRadius;
import static frc.robot.Constants.ShooterK.kRobotToTurret;
import static frc.robot.Constants.ShooterK.kTurretMaxAngle;
import static frc.robot.Constants.ShooterK.kTurretMinAngle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
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

import edu.wpi.first.units.measure.Time;

//TODO: make sure to explain everything
public class TurretCalculator {

    //see 5000's code (circa 2/16/2026 9:11 PM EST)
     public static final InterpolatingTreeMap<Double, ShotData> m_shotMap =
                new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);

        public static final InterpolatingDoubleTreeMap m_timeOfFlightMap = new InterpolatingDoubleTreeMap();

        static {
            m_shotMap.put(5.34, new ShotData(RotationsPerSecond.of(2900 / 60), Degrees.of(27)));
            m_timeOfFlightMap.put(5.34, 1.30);

            m_shotMap.put(4.90, new ShotData(RotationsPerSecond.of(2700 / 60), Degrees.of(26)));
            m_timeOfFlightMap.put(4.90, 1.42);

            m_shotMap.put(4.44, new ShotData(RotationsPerSecond.of(2820 / 60), Degrees.of(25.5)));
            m_timeOfFlightMap.put(4.44, 1.34);

            m_shotMap.put(4.05, new ShotData(RotationsPerSecond.of(2800 / 60), Degrees.of(25)));
            m_timeOfFlightMap.put(4.05, 1.36);

            m_shotMap.put(3.74, new ShotData(RotationsPerSecond.of(2750 / 60), Degrees.of(24)));
            m_timeOfFlightMap.put(3.74, 1.21);

            m_shotMap.put(3.42, new ShotData(RotationsPerSecond.of(2700 / 60), Degrees.of(23)));
            m_timeOfFlightMap.put(3.42, 1.40);

            m_shotMap.put(3.06, new ShotData(RotationsPerSecond.of(2610 / 60), Degrees.of(22)));
            m_timeOfFlightMap.put(3.06, 1.38);

            m_shotMap.put(2.73, new ShotData(RotationsPerSecond.of(2500 / 60), Degrees.of(20.5)));
            m_timeOfFlightMap.put(2.73, 1.34);

            m_shotMap.put(2.45, new ShotData(RotationsPerSecond.of(2450 / 60), Degrees.of(19.5)));
            m_timeOfFlightMap.put(2.45, 1.28);

            m_shotMap.put(2.14, new ShotData(RotationsPerSecond.of(2400 / 60), Degrees.of(18)));
            m_timeOfFlightMap.put(2.14, 1.31);

            m_shotMap.put(1.86, new ShotData(RotationsPerSecond.of(2350 / 60), Degrees.of(17)));
            m_timeOfFlightMap.put(1.86, 1.24);

            m_shotMap.put(1.55, new ShotData(RotationsPerSecond.of(2275 / 60), Degrees.of(15)));
            m_timeOfFlightMap.put(1.55, 1.23);
        }

    /**
     * Gets the Distance from current robot position to desired target.
     * Intended for shot calculation
     * @param robot current robot Pose
     * @param target desired target pose
     * @return the distance between the Robot and the Target
     */
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
    }

    // see https://www.desmos.com/geometry/l4edywkmha
    public static Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity, Translation3d target) {
        double gravity = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);
        double vel = velocity.in(InchesPerSecond);
        double x_dist = getDistanceToTarget(robot, target).in(Inches);
        double y_dist = target.getMeasureZ().minus(kRobotToTurret.getMeasureZ()).in(Inches);

        double angle = Math.atan(
            ((vel * vel) + Math.sqrt(Math.pow(vel, 4) - gravity * (gravity * x_dist * x_dist + 2 * y_dist * vel * vel)))
                / (gravity * x_dist));

        return Radians.of(angle);
    }


    //calculate how long it will take for a projectile to travel a certain amount of distance given its initial velocity and angle
    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        double vel = exitVelocity.in(MetersPerSecond);
        double angle = Math.PI / 2 - hoodAngle.in(Radians);
        double dist = distance.in(Meters);
        return Seconds.of(dist / (vel * Math.cos(angle)));
    }
    
    public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
    }

    /**
     * Calculates the turret's *TARGET* angle while ensuring it stays within physical limits.
     * IF the turret is near a limit, snaps 360 degrees in the opposite direction to reach the same angle
     * without hitting the hardstop.
     * @param robot used to calculate where on the robot the turret will be
     * @param target target position
     * @param currentAngle current measured position of the turret
     * @return safe rotation setpoint that is accurate to the target within bounds of kTurretMaxAngle
     * and kTurretMinAngle
     */
    public static Angle calculateAzimuthAngle(Pose2d robot, Translation3d target, Angle currentAngle) {
        Translation2d turretTranslation = new Pose3d(robot)
            .transformBy(kRobotToTurret)
            .toPose2d()
            .getTranslation();

        Translation2d direction = target.toTranslation2d().minus(turretTranslation);

        double angle = MathUtil.inputModulus(
                direction.getAngle().minus(robot.getRotation()).getRotations(), -0.5, 0.5);
        double current = currentAngle.in(Rotations);

        if (current > 0 && angle + 1 <= kTurretMaxAngle.in(Rotations))
            angle += 1;
        if (current < 0 && angle - 1 >= kTurretMinAngle.in(Rotations))
            angle -= 1;

        return Rotations.of(angle);
    }

    // Move a target a set time in the future along a velocity defined by fieldSpeeds
    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Translation3d(predictedX, predictedY, target.getZ());
    }

    // https://www.desmos.com/calculator/ezjqolho6g 
    // i lowkirkeniunely dont know what the difference between this desmos calc and the other
    public static ShotData calculateShotFromFunnelClearance(Pose2d robot, Translation3d actualTarget, Translation3d predictedTarget) {
        double x_dist = getDistanceToTarget(robot, predictedTarget).in(Inches);
        double y_dist = predictedTarget
            .getMeasureZ()
            .minus(kRobotToTurret.getMeasureZ())
            .in(Inches);
        double g = 386;
        double r = FieldConstants.Hub.funnelRadius.in(Inches) * x_dist / getDistanceToTarget(robot, actualTarget).in(Inches);
        double h = FieldConstants.Hub.funnelHeight.plus(kDistanceAboveFunnel).in(Inches); 
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
        
        return new ShotData(
                linearToAngularVelocity(InchesPerSecond.of(v0), kFlywheelRadius),
                Radians.of(Math.PI / 2 - theta),
                predictedTarget);

    }

    // use an iterative lookahead approach to determine shot parameters for a moving robot
    public static ShotData iterativeMovingShotFromFunnelClearance(
            Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
        // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
        ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
        Distance distance = getDistanceToTarget(robot, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        Translation3d predictedTarget = target;

        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(
                    shot.getExitVelocity(), shot.getHoodAngle(), getDistanceToTarget(robot, predictedTarget));
        }

        return shot;
    }

 
    /**
     * use an iterative interpolation approach to determine shot parameters for a moving robot 
     * compensating for speed, the target.
     * @param robot current robot pose
     * @param fieldSpeeds current robot speeds (w direction)
     * @param target target you are aiming for (either the passing point OR the HUB)
     * @param iterations amount of iterations to converge on one specific value
     * @return parameters to shoot a FUEL to the target accurately.
     */
    public static ShotData iterativeMovingShotFromInterpolationMap(
        Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
            double distance = getDistanceToTarget(robot, target).in(Meters);
            ShotData shot = m_shotMap.get(distance);
            shot = new ShotData(shot.exitVelocity, shot.hoodAngle, target);
            Time timeOfFlight = Seconds.of(m_timeOfFlightMap.get(distance));
            Translation3d predictedTarget = target; 

            //meant to iterate the process, and converge on one specific value.
            //gets a better ToF estimation & updates predictedTarget accordingly!
            for (int i = 0; i < iterations; i++) {
                predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
                distance = getDistanceToTarget(robot, predictedTarget).in(Meters);
                shot = m_shotMap.get(distance);
                shot = new ShotData(shot.exitVelocity, shot.hoodAngle, predictedTarget);
                timeOfFlight = Seconds.of(m_timeOfFlightMap.get(distance));
            }

            return shot;
    }


    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {
        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(RadiansPerSecond), hoodAngle.in(Radians), target);
        }

        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle) {
            this(exitVelocity, hoodAngle, FieldConstants.Hub.topCenterPoint);
        }

        public ShotData(double exitVelocity, double hoodAngle) {
            this(exitVelocity, hoodAngle, FieldConstants.Hub.topCenterPoint);
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
