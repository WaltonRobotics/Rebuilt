package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.util.AllianceFlipUtil;

public class Constants {
    public static final boolean kDebugLoggingEnabled = true;
    public static final double kSimPeriodicUpdateInterval = 0.020;

    public static final CANBus kCanivoreBus = new CANBus("fd");

    public static class ShooterK {
        public static final String kLogTab = "Shooter";

        /* MOTOR CONSTANTS */
        public static final double kFlywheelMoI = 0.000349 * 2.5;  //J for 5 3" 0.53lb flywheels
        public static final double kHoodMoI = 0.000249;
        public static final double kTurretMoI = 0.104506595;

        public static final double kFlywheelGearing = 1/1;
        public static final double kHoodGearing = 45/1;
        public static final double kTurretGearing = 41.66666666/1;

        public static final double kHoodLength = 0.153924; // hood circumference (49 degrees)
        public static final Angle kHoodMinDegs = Degrees.of(0);
        public static final Angle kHoodMaxDegs = Degrees.of(40);

        public static final Angle kTurretMaxRotsFromHome = Rotations.of(0.75); //0.75 rots in each direction from home
        public static final Angle kTurretMinRots = Rotations.of(-kTurretMaxRotsFromHome.magnitude());
        public static final Angle kTurretMaxRots = Rotations.of(kTurretMaxRotsFromHome.magnitude());

        public static final AngularVelocity kFlywheelMaxRPS = RotationsPerSecond.of(5785/60 * (0.9));   //Kraken X60Foc Max (RPM: 5785)
        public static final AngularVelocity kFlywheelEmergencyRPS = RotationsPerSecond.of(1500/60 * (0.9));
        public static final AngularVelocity kFlywheelZeroRPS = RotationsPerSecond.of(/* 0/60 * (0.9) */ 0);

        /* IDS */
        public static final int kLeaderCANID = 20;
        public static final int kFollowerCANID = 21;
        public static final int kTurretCANID = 12;

        public static final int kExitBeamBreakChannel = 0; //TODO: Update channel number
        public static final int kHoodChannel = 1;
        public static final int kHoodEncoderChannel = 2;

        /* CONFIGS */
        // TODO: Check what more configs would be necessary
        private static final Slot0Configs kFlywheelLeaderSlot0Configs = new Slot0Configs()   //Note to self (hrehaan) (and saarth cuz i did the same thing): the default PID sets ZERO volts to a motor, which makes all sim effectively useless cuz the motor has ZERO supplyV
            .withKS(0)
            .withKV(0.1217)
            .withKA(0)
            .withKP(0)
            .withKI(0)
            .withKD(0); // kP was causing the werid sinusoid behavior, kS and kA were adding inconsistency with the destination values
        private static final CurrentLimitsConfigs kFlywheelLeaderCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true);
        private static final MotorOutputConfigs kFlywheelLeaderOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: check whether this should be CW or CCW
            .withNeutralMode(NeutralModeValue.Brake);
        private static final FeedbackConfigs kFlywheelFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kFlywheelGearing);
        public static final TalonFXConfiguration kFlywheelLeaderTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kFlywheelLeaderSlot0Configs)
            .withCurrentLimits(kFlywheelLeaderCurrentLimitConfigs)
            .withMotorOutput(kFlywheelLeaderOutputConfigs)
            .withFeedback(kFlywheelFeedbackConfigs);
        
        // TODO: mimics the leader, so it doesn't need its own configs - right?
        public static final TalonFXConfiguration kFlywheelFollowerTalonFXConfiguration = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive) //TODO: check whether this should be CW or CCW
                    .withNeutralMode(NeutralModeValue.Brake));

        // TODO: I assume we would want the Hood and Turret to move at a constant high velocity
        //       so we should probably configure that here?
        //
        //       that's not exactly how it works - i'm not sure either but kV is more like a boost to velo than velo itself
        //       so in cases like the turret there's no kV
        private static final Slot0Configs kHoodSlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(1)
            .withKA(0)
            .withKP(0.85)
            .withKI(0.1)
            .withKD(0); // kP was too low making the slope less steep, kS and kA were adding weird behavior, added kI to account for kP overshooting
        private static final CurrentLimitsConfigs kHoodCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(110)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimitEnable(true);
        private static final MotorOutputConfigs kHoodOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: check whether this should be CW or CCW
            .withNeutralMode(NeutralModeValue.Brake);
        private static final FeedbackConfigs kHoodFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kHoodGearing);
        public static final TalonFXConfiguration kHoodTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kHoodSlot0Configs)
            .withCurrentLimits(kHoodCurrentLimitConfigs)
            .withMotorOutput(kHoodOutputConfigs)
            .withFeedback(kHoodFeedbackConfigs);
        
        private static final Slot0Configs kTurretSlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(4.07)
            .withKA(0.02)
            .withKP(120)  //3 - testing values in Pheonix Tuner
            .withKI(0)
            .withKD(0); // OLD: kP was too low making the slope less steep, kS kV and kA were causing rlly weird behavior (jumping up/down way further than targeted position)
        private static final CurrentLimitsConfigs kTurretCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withSupplyCurrentLimit(20)
            .withStatorCurrentLimitEnable(true);
        private static final MotorOutputConfigs kTurretOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: check whether this should be CW or CCW
            .withNeutralMode(NeutralModeValue.Brake)
            .withPeakForwardDutyCycle(0.1)
            .withPeakReverseDutyCycle(0.1);
        private static final MotionMagicConfigs kTurretMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(110)  //TODO: update MMV Configs
            .withMotionMagicAcceleration(20)
            .withMotionMagicJerk(0);
        private static final SoftwareLimitSwitchConfigs kTurretSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(0.78)    //TODO: update threshold numbers
            .withReverseSoftLimitEnable(false)
            .withReverseSoftLimitThreshold(0.02);
        private static final FeedbackConfigs kTurretFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kTurretGearing);
        public static final TalonFXConfiguration kTurretTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kTurretSlot0Configs)
            .withCurrentLimits(kTurretCurrentLimitConfigs)
            .withMotorOutput(kTurretOutputConfigs)
            .withMotionMagic(kTurretMotionMagicConfigs)
            .withSoftwareLimitSwitch(kTurretSoftwareLimitSwitchConfigs)
            .withFeedback(kTurretFeedbackConfigs);

        public static final CanandmagSettings kHoodEncoderSettings = new CanandmagSettings()
            .setInvertDirection(false);

        //Left, Center (Climb), Center (Hub), Right - Driver POV
        public static final Pose2d kShooterOverridePose[] = {
            AllianceFlipUtil.apply(new Pose2d(FieldK.kFieldLengthMeters / 6, FieldK.kFieldWidthMeters * 2 / 3, new Rotation2d(0))),
            AllianceFlipUtil.apply(new Pose2d(Units.inchesToMeters(156.61 - 115.05 + 10), FieldK.kFieldWidthMeters / 2, new Rotation2d(0))),
            AllianceFlipUtil.apply(new Pose2d(Units.inchesToMeters(156.61 - 10), FieldK.kFieldWidthMeters / 2, new Rotation2d(0))),
            AllianceFlipUtil.apply(new Pose2d(FieldK.kFieldLengthMeters / 6, FieldK.kFieldWidthMeters / 3, new Rotation2d(0))),
        };
    }

    public static class VisionK {
        public static final Camera[] kCameras = new Camera[4];
        private static final String kSimCameraSimVisualNames = "VisionEstimation"; //suffixed to each camera name

        //Initialize cameras
        static {
            kCameras[0] = new Camera(
                new SimCameraProperties(), 
                "frontLeftCamera", 
                kSimCameraSimVisualNames, 
                Camera.transformToRobo(0, 0, 0, 0, 0, 0) //TODO:
            );
            kCameras[0].setProps("ThriftyCam", 0, 0, 0, 0);
            
            kCameras[1] = new Camera(
                new SimCameraProperties(), 
                "frontRightCamera", 
                kSimCameraSimVisualNames, 
                Camera.transformToRobo(0, 0, 0, 0, 0, 0)
            );
            kCameras[1].setProps("ThriftyCam", 0, 0, 0, 0);

            kCameras[2] = new Camera(
                new SimCameraProperties(), 
                "backLeftCamera", 
                kSimCameraSimVisualNames, 
                Camera.transformToRobo(0, 0, 0, 0, 0, 0)
            );
            kCameras[2].setProps("ThriftyCam", 0, 0, 0, 0);

            kCameras[3] = new Camera(
                new SimCameraProperties(), 
                "backRightCamera", 
                kSimCameraSimVisualNames, 
                Camera.transformToRobo(0, 0, 0, 0, 0, 0)
            );
            kCameras[3].setProps("ThriftyCam", 0, 0, 0, 0);
        }
    }

    public static class FieldK {
        // take with a grain of salt - pulled from field dimensions (welded)
        public static final double kFieldLengthMeters = Units.inchesToMeters(651.22); 
        public static final double kFieldWidthMeters = Units.inchesToMeters(317.69);

        public static final AprilTagFieldLayout kTagLayout;

        //Ignore trench April Tags
        static {
            HashSet<Integer> excludedAprilTagsID = new HashSet<> (Arrays.asList(1, 6, 7, 12, 17, 22, 23, 28));
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            List<AprilTag> tags = new ArrayList<> (fieldLayout.getTags());
            tags.removeIf(tag -> excludedAprilTagsID.contains(tag.ID));
            kTagLayout = new AprilTagFieldLayout(tags, fieldLayout.getFieldLength(), fieldLayout.getFieldWidth());
        }
    }

    public static class RobotK {
        public static final String kLogTab = "Superstructure";
    }

    public static class IntakeK {
        public static final String kLogTab = "Intake";

        /* MOTOR CONSTANTS */
        public static final double kDeployMOI = 0.0209;
        public static final double kDeployGearing = 5/1;

        public static final double kRollersMOI = 0.00005;
        public static final double kRollersGearing = 1.0/2;

        /* IDS */
        public static final int kDeployCANID = 40;
        public static final int kRollersCANID = 41;

        /* CONFIGS */
        //Deploy Motor
        private static final CurrentLimitsConfigs kDeployCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(20)
            .withStatorCurrentLimitEnable(true);
        private static final Slot0Configs kDeploySlot0Configs = new Slot0Configs()
            .withKS(0.04)
            .withKV(0)
            .withKA(0)
            .withKP(0.8)
            .withKI(0)
            .withKD(0);
        public static final MotorOutputConfigs kDeployMotorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);
        private static final MotionMagicConfigs kDeployMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(100)
            .withMotionMagicJerk(0);
        public static final FeedbackConfigs kDeployFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(1 / kDeployGearing);
        public static final TalonFXConfiguration kDeployConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kDeployCurrentLimitConfigs)
            .withSlot0(kDeploySlot0Configs)
            .withMotorOutput(kDeployMotorOutputConfigs)
            .withMotionMagic(kDeployMotionMagicConfigs);

        //Rollers Motor
        private static final CurrentLimitsConfigs kRollersCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final Slot0Configs kRollersSlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(0.19543973941)
            .withKA(0)
            .withKP(0)
            .withKI(0)
            .withKD(0);
        public static final MotorOutputConfigs kRollersMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // TODO: CW or CCW?
            .withNeutralMode(NeutralModeValue.Brake);
        public static final FeedbackConfigs kRollersFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(1 / kRollersGearing);
        public static final TalonFXConfiguration kRollersConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kRollersCurrentLimitConfigs)
            .withSlot0(kRollersSlot0Configs)
            .withMotorOutput(kRollersMotorOutputConfigs)
            .withFeedback(kRollersFeedbackConfigs);
    }

    public static class IndexerK {
        public static String kLogTab = "Indexer";
        
        /* IDS */
        public static final int kSpinnerCANID = 10;
        public static final int kExhaustCANID = 11;

        public static final double kSpinnerGearing = 3;
        public static final double kExhaustGearing = 1/1.2;

        public static final double kSpinnerMOI = 0.00166190059;
        public static final double kExhaustMOI = 0.000215968064;
        
        /* CONFIGS */
        //TODO: Make transfer configs accurate
        private static final Slot0Configs kSpinnerSlot0Configs = new Slot0Configs()
            .withKS(0.012)
            .withKV(0.371)
            .withKA(0)
            .withKP(0.1)
            .withKI(0)
            .withKD(0.03);
        private static final CurrentLimitsConfigs kSpinnerCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(70)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kSpinnerMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: CW or CCW?
            .withNeutralMode(NeutralModeValue.Brake);
        private static final FeedbackConfigs kSpinnerFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kSpinnerGearing);
        public static final TalonFXConfiguration kSpinnerTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kSpinnerSlot0Configs)
            .withCurrentLimits(kSpinnerCurrentLimitConfigs)
            .withMotorOutput(kSpinnerMotorOutputConfigs)
            .withFeedback(kSpinnerFeedbackConfigs);

        private static final Slot0Configs kExhaustSlot0Configs = new Slot0Configs()
            .withKS(0.1124)
            .withKV(0.102)
            .withKA(0)
            .withKP(0.06)
            .withKI(0)
            .withKD(0);
        private static final CurrentLimitsConfigs kExhaustCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(140)
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kExhaustMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: CW or CCW?
            .withNeutralMode(NeutralModeValue.Brake);
        private static final FeedbackConfigs kExhaustFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kExhaustGearing);
        public static final TalonFXConfiguration kExhaustTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kExhaustSlot0Configs)
            .withCurrentLimits(kExhaustCurrentLimitConfigs)
            .withMotorOutput(kExhaustMotorOutputConfigs)
            .withFeedback(kExhaustFeedbackConfigs);
    }

    public static class AutonK {
        public static final Pose2d neutralPose = new Pose2d(Distance.ofRelativeUnits(6.924767017364502, Meter), 
            Distance.ofRelativeUnits(2.251265048980713, Meter), new Rotation2d(Math.PI));
        public static final Pose2d depotPose = new Pose2d(Distance.ofRelativeUnits(1.1576627492904663, Meter), 
            Distance.ofRelativeUnits(5.958622932434082, Meter), new Rotation2d(0));
    }
}
