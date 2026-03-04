package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.util.AllianceFlipALWAYSUtil;
import frc.util.AllianceFlipUtil;
import frc.util.VisionUtil;
import frc.util.WaltLogger;
import frc.util.WaltLogger.Pose3dLogger;

public class Constants {
    public static final boolean kDebugLoggingEnabled = true;
    public static final double kSimPeriodicUpdateInterval = 0.020;

    public static final CANBus kCanivoreBus = new CANBus("fd");

    public static class WpiK {
        public static final ChassisSpeeds kZeroChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }

    public static class ShooterK {
        public static final String kLogTab = "Shooter";
        private static final Rotation2d kTurretAngleOffset = Rotation2d.fromDegrees(-135);
        public static final Transform3d kTurretTransform = new Transform3d(new Translation3d(Inches.of(-4.744), Inches.of(-4.239), Inches.of(17.260)), new Rotation3d(kTurretAngleOffset)); //DUMMY VALS
        public static final Distance kInchesAboveFunnel = Inches.of(20);// distance the ball must travel above the funnel opening to arc correctly into the hub

        private static final Pose3dLogger log_turretTransform = WaltLogger.logPose3d(kLogTab, "TurretTransformRaw");
        static {
            log_turretTransform.accept(kTurretTransform);
        }

        public static final Distance kFlywheelRadius = Inches.of(1.5); 

        public static final int kHopperCapacity = 55; //TODO: find true max

        public static final double kGravity = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);

        //TODO: work out where our passing spots should be..?
        public static final Translation3d kPassingSpotLeft = new Translation3d(
                Inches.of(90), FieldConstants.fieldWidthIn.div(2).plus(Inches.of(105)), Inches.zero());
        // public static final Translation3d kPassingSpotCenter = new Translation3d(
        //         Inches.of(90), FieldConstants.fieldWidthIn.div(2), Inches.zero());
        public static final Translation3d kPassingSpotRight = new Translation3d(
            Inches.of(90), Inches.of(AllianceFlipALWAYSUtil.applyY(kPassingSpotLeft.getY())), Inches.of(0));

        /* MOTOR CONSTANTS */
        public static final double kShooterMoI = 0.000349 * 2.5;  //J for 5 3" 0.53lb flywheels
        public static final double kTurretMoI = 0.104506595;

        public static final double kShooterGearing = 1/1;
        public static final double kTurretGearing = 41.66666666/1;

        public static final int kPeakShooterVolts = 16;

        public static final Angle kTurretMaxRotsFromHome = Rotations.of(0.75); //0.75 rots in each direction from home
        public static final Angle kTurretMinRots = Rotations.of(-kTurretMaxRotsFromHome.magnitude());
        public static final Angle kTurretMaxRots = Rotations.of(kTurretMaxRotsFromHome.magnitude());

        public static final AngularVelocity kShooterRPS = RotationsPerSecond.of((5785/60) * (0.65) / kShooterGearing);   //Kraken X60Foc Max (RPM: 5785) //(0.9)
        public static final AngularVelocity kShooterBarfRPS = RotationsPerSecond.of((5785/60) * (0.2) / kShooterGearing);
        public static final AngularVelocity kShooterZeroRPS = RotationsPerSecond.of(/* 0/60 * (0.9) / kShooterGearing */ 0);

        //---HOOD CONSTANTS
        public static final double kHoodMoI = 0.00027505;
        public static final double kHoodEncoderGearing = 360/40.0;

        // 300° on the servo is 0° on the hood, and 0° on the servo is 40° on the hood.
        // servo to hood: 300 : 0 || 0 : 40
        // hood to encoder: 0 : 0 || 40 : 0.9451 (340.236)
        // servo to encoder: 300 : 0 || 0 : 0.9451 (340.236)
        public static final Angle kHoodMinDegs = Degrees.of(0); // 0 = 0 (encoder wise i believe)
        public static final Angle kHoodSafeDegs = Degrees.of(1);
        public static final Angle kHoodMaxDegs = Degrees.of(37);    // 40 = 0.9451 (encoder wise i believe)
        public static final Angle kHoodEncoderMaxDegs = Degrees.of(Rotations.of(0.9451).in(Degrees));
        public static final Angle kHoodAbsoluteMaxDegs = Degrees.of(40);
        public static final Angle kHoodServoMaxDegs = Degrees.of(300);

        public static final Angle kHoodShootingTolerance = Degrees.of(0.5);

        public static final DCMotor khoodDCMotorGearbox = new DCMotor(
            6, 
            0.047, 
            2.5, 
            0.2, 
            24.0855, 
            1
        );

        /* IDS */
        public static final int kShooterA_CANID = 20;
        public static final int kShooterB_CANID = 21;
        public static final int kTurretCANID = 12;

        // public static final int kExitBeamBreakChannel = 1; //TODO: Update channel number
        public static final int kHoodChannel = 0;
        public static final int kHoodEncoderCANID = 25;

        /* CONFIGS */
        // TODO: Check what more configs would be necessary
        private static final Slot0Configs kShooterASlot0Configs = new Slot0Configs()   //Note to self (hrehaan) (and saarth cuz i did the same thing): the default PID sets ZERO volts to a motor, which makes all sim effectively useless cuz the motor has ZERO supplyV
            .withKS(0)
            .withKV(0.12)
            .withKA(0)
            .withKP(0.5)
            .withKI(0)
            .withKD(0); // kP was causing the werid sinusoid behavior, kS and kA were adding inconsistency with the destination values
        private static final CurrentLimitsConfigs kShooterACurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true);
        private static final MotorOutputConfigs kShooterAOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        private static final FeedbackConfigs kShooterAFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kShooterGearing);
        private static final VoltageConfigs kShooterAVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(kPeakShooterVolts)
            .withPeakReverseVoltage(-kPeakShooterVolts);
        public static final TalonFXConfiguration kShooterATalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kShooterASlot0Configs)
            .withCurrentLimits(kShooterACurrentLimitConfigs)
            .withMotorOutput(kShooterAOutputConfigs)
            .withFeedback(kShooterAFeedbackConfigs)
            .withVoltage(kShooterAVoltageConfigs);

        private static final MotorOutputConfigs kShooterBOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
        public static final TalonFXConfiguration kShooterBTalonFXConfiguration = new TalonFXConfiguration()
                .withCurrentLimits(kShooterACurrentLimitConfigs)
                .withMotorOutput(kShooterBOutputConfigs)
                .withFeedback(kShooterAFeedbackConfigs)
                .withVoltage(kShooterAVoltageConfigs);

        //---HOOD
        private static final MagnetSensorConfigs kHoodEncoderMagnetSensorConfigs = new MagnetSensorConfigs()
            .withMagnetOffset(Rotations.of(-0.0068359375))
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1));
        public static final CANcoderConfiguration kHoodEncoderConfiguration = new CANcoderConfiguration()
            .withMagnetSensor(kHoodEncoderMagnetSensorConfigs);

        //---TURRET
        private static final Slot0Configs kTurretSlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(5)
            .withKA(0.02)
            .withKP(300)  //3 - testing values in Pheonix Tuner
            .withKI(0)
            .withKD(5); // OLD: kP was too low making the slope less steep, kS kV and kA were causing rlly weird behavior (jumping up/down way further than targeted position)
        private static final CurrentLimitsConfigs kTurretCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(55)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(55)
            .withSupplyCurrentLowerLimit(15)
            .withSupplyCurrentLowerTime(1.0) // drop to 15A after 1 second
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kTurretOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        private static final MotionMagicConfigs kTurretMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(110)  //TODO: update MMV Configs
            .withMotionMagicAcceleration(20)
            .withMotionMagicJerk(0);
        private static final SoftwareLimitSwitchConfigs kTurretSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0.75)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(-0.75);
        private static final FeedbackConfigs kTurretFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kTurretGearing);
        private static final VoltageConfigs kTurretVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12);
        public static final TalonFXConfiguration kTurretTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kTurretSlot0Configs)
            .withCurrentLimits(kTurretCurrentLimitConfigs)
            .withMotorOutput(kTurretOutputConfigs)
            .withMotionMagic(kTurretMotionMagicConfigs)
            .withSoftwareLimitSwitch(kTurretSoftwareLimitSwitchConfigs)
            .withFeedback(kTurretFeedbackConfigs)
            .withVoltage(kTurretVoltageConfigs);

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
        // public static final Camera[] kCameras = new Camera[4];
        // private static final String kSimCameraSimVisualNames = /"VisionEstimation"; //suffixed to each camera name

        public static final Transform3d kFrontLeftCTR = VisionUtil.transformToRobo(10.413, 12.394, 28.844, 0, -10, 45);
        public static final Transform3d kFrontRightCTR = VisionUtil.transformToRobo(10.413, -12.394, 28.844, 0, -10, 315);
        public static final Transform3d kBackLeftCTR = VisionUtil.transformToRobo(-11.894, 12.394, 28.844, 0, -10, 135);
        public static final Transform3d kBackRightCTR = VisionUtil.transformToRobo(-11.894, -12.394, 28.844, 0, -10, 225);
        //Initialize cameras
        // static {
        //     kCameras[0] = new Camera(
        //         new SimCameraProperties(), 
        //         "FrontLeft", 
        //         kSimCameraSimVisualNames, 
        //         Camera.transformToRobo(10.413, 12.394, 28.844, 0, -10, 45)
        //     );
        //     kCameras[0].setProps("ThriftyCam", 0, 0, 0, 0);
            
        //     kCameras[1] = new Camera(
        //         new SimCameraProperties(), 
        //         "FrontRight", 
        //         kSimCameraSimVisualNames, 
        //         Camera.transformToRobo(10.413, -12.394, 28.844, 0, -10, 315)
        //     );
        //     kCameras[1].setProps("ThriftyCam", 0, 0, 0, 0);

        //     kCameras[2] = new Camera(
        //         new SimCameraProperties(), 
        //         "BackLeft", 
        //         kSimCameraSimVisualNames, 
        //         Camera.transformToRobo(-11.894, 12.394, 28.844, 0, -10, 135)
        //     );
        //     kCameras[2].setProps("ThriftyCam", 0, 0, 0, 0);

        //     kCameras[3] = new Camera(
        //         new SimCameraProperties(), 
        //         "BackRight", 
        //         kSimCameraSimVisualNames, 
        //         Camera.transformToRobo(-11.894, -12.394, 28.844, 0, -10, 225)
        //     );
        //     kCameras[3].setProps("ThriftyCam", 0, 0, 0, 0);
        // }
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
        public static final String kLogTab = "Robot";

        public static final int kMiniPCChannel = 14;

        // real values
        public static final Distance kRobotFullWidth = Inches.of(33.6875);
        public static final Distance kRobotFullLength = Inches.of(32.6875);
        public static final Distance kBumperHeight = Inches.of(4.5);
        public static final double kRobotSpeedIntakingLimit = 0.13;
    }

    public static class SuperstructureK {
        public static final String kLogTab = "Superstructure";
    }

    public static class IntakeK {
        public static final String kLogTab = "Intake";

        /* MOTOR CONSTANTS */
        public static final double kIntakeArmMOI = 0.0209;
        public static final double kIntakeArmGearing = 125/1;

        public static final double kIntakeRollersMOI = 0.0001; // 0.00343880857
        public static final double kIntakeRollersGearing = 12.0/30;

        public static final AngularVelocity kIntakeRollersMaxRPS = RotationsPerSecond.of((5785 / 60) / kIntakeRollersGearing * .8);  //100% RPS

        /* IDS */
        public static final int kIntakeArmCANID = 40;
        public static final int kIntakeRollersCANID = 41;

        /* CONFIGS */
        //IntakeArm Motor
        private static final CurrentLimitsConfigs kIntakeArmCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final Slot0Configs kIntakeArmSlot0Configs = new Slot0Configs()
            .withKS(1.5)
            .withKV(0)
            .withKA(0)
            .withKP(50)
            .withKI(0)
            .withKD(0);
        public static final MotorOutputConfigs kIntakeArmMotorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        private static final MotionMagicConfigs kIntakeArmMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(0.75)
            .withMotionMagicAcceleration(10)
            .withMotionMagicJerk(0);
        public static final FeedbackConfigs kIntakeArmFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kIntakeArmGearing);
        private static final VoltageConfigs kIntakeArmVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12);
        public static final TalonFXConfiguration kIntakeArmConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kIntakeArmCurrentLimitConfigs)
            .withSlot0(kIntakeArmSlot0Configs)
            .withMotorOutput(kIntakeArmMotorOutputConfigs)
            .withMotionMagic(kIntakeArmMotionMagicConfigs)
            .withVoltage(kIntakeArmVoltageConfigs)
            .withFeedback(kIntakeArmFeedbackConfigs);

        //Intake Rollers Motor
        private static final CurrentLimitsConfigs kIntakeRollersCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final Slot0Configs kIntakeRollersSlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(0.488599348534) // 0.19543973941
            .withKA(0)
            .withKP(0)
            .withKI(0)
            .withKD(0);
        public static final MotorOutputConfigs kIntakeRollersMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        public static final FeedbackConfigs kIntakeRollersFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kIntakeRollersGearing);
        private static final VoltageConfigs kIntakeRollersVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(12)    //1.2
            .withPeakReverseVoltage(-12);  //-1.2
        public static final TalonFXConfiguration kIntakeRollersConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kIntakeRollersCurrentLimitConfigs)
            .withSlot0(kIntakeRollersSlot0Configs)
            .withMotorOutput(kIntakeRollersMotorOutputConfigs)
            .withFeedback(kIntakeRollersFeedbackConfigs)
            .withVoltage(kIntakeRollersVoltageConfigs);
    }

    public static class IndexerK {
        public static String kLogTab = "Indexer";
        
        /* IDS */
        //TODO: Make ids accurate
        public static final int kSpindexerCANID = 10;
        public static final int kTunnelCANID = 11;

        public static final double kSpindexerGearing = 3;
        public static final double kTunnelGearing = 1/1.2 * 0.5;

        public static final double kSpindexerMOI = 0.00166190059;
        public static final double kTunnelMOI = 0.000215968064;
      
        public static final AngularVelocity kSpindexerIntakeRPS = RotationsPerSecond.of((5785/60) * (-0.20) / kSpindexerGearing);  //Max RPM for X60Foc is 5785   (0.9)
        public static final AngularVelocity kSpindexerRPS = RotationsPerSecond.of((5785/60) * (0.45) / kSpindexerGearing);  //Max RPM for X60Foc is 5785   (0.9)
        public static final AngularVelocity kTunnelRPS = RotationsPerSecond.of((5785/60) * (0.60) / kTunnelGearing);   //(0.9) //(0.65)
        
        /* CONFIGS */
        //TODO: Make transfer configs accurate
        private static final Slot0Configs kSpindexerSlot0Configs = new Slot0Configs()
            .withKS(0.012)
            .withKV(0.371)
            .withKA(0)
            .withKP(0.1)
            .withKI(0)
            .withKD(0.03);
        private static final CurrentLimitsConfigs kSpindexerCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(65)
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLowerTime(0)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kSpindexerMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: CW or CCW?
            .withNeutralMode(NeutralModeValue.Coast);
        private static final FeedbackConfigs kSpindexerFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kSpindexerGearing);
        private static final VoltageConfigs kSpindexerVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(6)    //1.2
            .withPeakReverseVoltage(-6);  //-1.2
        public static final TalonFXConfiguration kSpindexerTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kSpindexerSlot0Configs)
            .withCurrentLimits(kSpindexerCurrentLimitConfigs)
            .withMotorOutput(kSpindexerMotorOutputConfigs)
            .withFeedback(kSpindexerFeedbackConfigs)
            .withVoltage(kSpindexerVoltageConfigs);

        private static final Slot0Configs kTunnelSlot0Configs = new Slot0Configs()
            .withKS(0.1124)
            .withKV(0.102)
            .withKA(0)
            .withKP(0.06)
            .withKI(0)
            .withKD(0);
        private static final CurrentLimitsConfigs kTunnelCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kTunnelMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        private static final FeedbackConfigs kTunnelFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kTunnelGearing);
        private static final VoltageConfigs kTunnelVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(6)  //1.2
            .withPeakReverseVoltage(-6);    //-1.2
        public static final TalonFXConfiguration kTunnelTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kTunnelSlot0Configs)
            .withCurrentLimits(kTunnelCurrentLimitConfigs)
            .withMotorOutput(kTunnelMotorOutputConfigs)
            .withFeedback(kTunnelFeedbackConfigs)
            .withVoltage(kTunnelVoltageConfigs);
    }

    public static class AutonK {
        public static final Pose2d rightNeutralPose = new Pose2d(Meters.of(6.924767017364502), 
            Meters.of(2.251265048980713), new Rotation2d(0));
        public static final Pose2d rightDepotPose = new Pose2d(Meters.of(1.1576627492904663), 
            Meters.of(5.958622932434082), new Rotation2d(Math.PI));

        public static final Pose2d leftNeutralPose = new Pose2d(Meters.of(6.924767017364502), 
            Meters.of(5.437880039215088), new Rotation2d(0));

    }
}
