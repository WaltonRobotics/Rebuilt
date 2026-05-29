
package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.IndexerK.kSpindexerShootRPSD;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.util.AllianceFlipUtil;
import frc.util.VisionUtil;

/*
 * Constants
 *
 * This file is the single source of truth for every tunable value, hardware ID,
 * and configuration object in the robot code. Keeping everything here makes it
 * easy to update a value in one place without hunting through multiple files.
 *
 * It's organized into nested static classes, one per subsystem or topic:
 *   MotorK         — motor physical constants (free speeds, used for RPS calculations)
 *   WpiK           — WPILib utility constants
 *   ShooterK       — flywheel, turret, and hood settings + TalonFX configs
 *   VisionK        — camera positions relative to the robot
 *   FieldK         — field dimensions and April Tag layout
 *   RobotK         — robot physical dimensions and misc settings
 *   SuperstructureK — log tab name for Superstructure
 *   IntakeK        — intake arm + roller settings + TalonFX configs
 *   IndexerK       — spindexer + tunnel settings, speed ratios, TalonFX configs
 *   TurretK        — turret encoder offsets and gear tooth counts
 *   AutonK         — auton timeouts and Choreo trajectory file names
 *
 * Quick note on CTRE motor configs:
 *   Each motor has a TalonFXConfiguration object built up from sub-configs.
 *   The PID gains (Slot0Configs) use this terminology:
 *     kS — static friction compensation (minimum voltage to move the motor at all)
 *     kV — velocity feedforward (voltage per RPS)
 *     kA — acceleration feedforward (voltage per RPS/s)
 *     kP — proportional gain (more error → more correction)
 *     kI — integral gain (corrects sustained steady-state error over time)
 *     kD — derivative gain (dampens oscillation)
 *   SensorToMechanismRatio in FeedbackConfigs accounts for gear reduction so the
 *   motor controller reports mechanism position/speed directly, not motor shaft position.
 *
 * CAN bus note:
 *   kRioBus    — default RoboRIO CAN bus (lower bandwidth, used for intake)
 *   kCanivoreBus — high-speed CANivore bus ("fd"), used for indexer + drivetrain
 *   kShooterBus  — dedicated CAN bus for the shooter subsystem ("shooter")
 */
public class Constants {
    // Global feature flags — toggle these without touching subsystem code.
    public static final boolean kDebugLoggingEnabled = false;
    public static final boolean kDataLoggingEnabled = true;
    public static final boolean kUsePoseCorrection = false;
    public static final double kSimPeriodicUpdateInterval = 0.020;

    // CAN bus identifiers. Motors are assigned to specific buses based on bandwidth needs.
    public static final CANBus kRioBus = CANBus.roboRIO();
    public static final CANBus kCanivoreBus = new CANBus("fd");
    public static final CANBus kShooterBus = new CANBus("shooter");


    // =============================================================
    // MOTOR CONSTANTS
    // Free speed is the no-load max speed of each motor variant.
    // These are used to compute max RPS for velocity scaling.
    // =============================================================
    public static final class MotorK {
        public static final double kX60MaxRadPerSec = DCMotor.getKrakenX60(1).freeSpeedRadPerSec;
        public static final AngularVelocity kX60MaxVelocity = RadiansPerSecond.of(kX60MaxRadPerSec);
        public static final double kX60FOCMaxRadPerSec = DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec;
        public static final AngularVelocity kX60FOCMaxVelocity = RadiansPerSecond.of(kX60FOCMaxRadPerSec);

        public static final double kX44MaxRadPerSec = DCMotor.getKrakenX44(1).freeSpeedRadPerSec;
        public static final AngularVelocity kX44MaxVelocity = RadiansPerSecond.of(kX44MaxRadPerSec);
        public static final double kX44FOCMaxRadPerSec = DCMotor.getKrakenX44Foc(1).freeSpeedRadPerSec;
        public static final AngularVelocity kX44FOCMaxVelocity = RadiansPerSecond.of(kX44FOCMaxRadPerSec);
    }


    // =============================================================
    // WPILib UTILITY CONSTANTS
    // =============================================================
    public static class WpiK {
        public static final ChassisSpeeds kZeroChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }


    // =============================================================
    // SHOOTER CONSTANTS
    // Flywheel, turret, and hood settings.
    // =============================================================
    public static class ShooterK {
        public static final String kLogTab = "Shooter";

        // ---- turret geometry ----
        // The turret is offset from the robot's center. These constants describe
        // where it is and how it's rotated relative to the robot frame.
        // kTurretAngleOffset corrects for the turret's mechanical zero not being
        // aligned with the robot's forward direction.
        public static final Rotation2d kTurretAngleOffset = Rotation2d.fromRotations(0.106 + 0.0067);
        public static final Rotation3d kTurretAngleOffset3d = new Rotation3d(kTurretAngleOffset);
        public static final Translation3d kTurretTranslation = new Translation3d(Inches.of(-4.744), Inches.of(-4.239), Inches.of(15.769));
        public static final Transform3d kTurretTransformNoRotation = new Transform3d(kTurretTranslation, Rotation3d.kZero);
        public static final Transform3d kTurretTransform = new Transform3d(kTurretTranslation, kTurretAngleOffset3d);

        // How far above the funnel opening the ball must travel to arc correctly into the hub.
        public static final Distance kInchesAboveFunnel = Inches.of(20);

        // Turret position used when "barfing" (shooting at low speed to clear jams).
        public static final Angle kTurretBarfPos = Rotations.of(-0.113);

        public static final boolean kUseStaticShot = false;
        public static final boolean kAllowDriverRPSTweak = false;

        public static final Distance kFlywheelRadius = Inches.of(1.5);

        // Pre-computed doubles for the shot calculator hot path.
        // Using doubles directly avoids creating Measure objects on every loop iteration.
        public static final double kTurretOffsetX_m = kTurretTransform.getTranslation().getX();
        public static final double kTurretOffsetY_m = kTurretTransform.getTranslation().getY();
        public static final double kTurretAngleOffsetRad = kTurretAngleOffset.getRadians();
        public static final double kFlywheelRadiusM = kFlywheelRadius.in(Meters);
        public static final double kFlywheelRadiusIn = kFlywheelRadius.in(Inches);
        public static final double kTurretOffsetZ_in = kTurretTransform.getTranslation().getMeasureZ().in(Inches);
        public static final double kFunnelRadiusIn = FieldConstants.Hub.funnelRadius.in(Inches);
        public static final double kFunnelHeightPlusAboveIn = FieldConstants.Hub.funnelHeight.plus(kInchesAboveFunnel).in(Inches);

        // Lateral bias compensation: balls drift left/right as a function of turret angle.
        // sin(turretRelAngle) = 0 at 0/180°, +1 at 90° (left bias), -1 at 270° (right bias).
        // This gain (in rotations) is subtracted * sin to counter the drift. Tune on robot.
        public static final double kTurretLateralBiasGainRots = 0;

        public static final int kHopperCapacity = 55; //TODO: find true max

        public static final double kGravity = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);

        // Passing zone: the X coordinate beyond which the robot can pass to a partner.
        public static final Distance kPassingX = Meters.of(3.5);
        public static final double kPassingXAsDouble = kPassingX.in(Meters);

        // No-pass zone: a region around the hub where passing is blocked.
        public static final double kNoPassZoneTopX = FieldConstants.Hub.blueInnerCenterPoint.getX() + 2;
        public static final double kNoPassZoneRightY = Meters.of(3.2).baseUnitMagnitude();
        public static final double kNoPassZoneLeftY = FieldConstants.fieldWidth - 3.2;

        public static final double kShooterTimeout = 1.0;
        // How long ball detection must be triggered before we count it as a real shot (debounce).
        public static final double kBallDetectedDebounceTime = 1.2;

        // ---- motor constants ----
        public static final double kShooterMoI = 0.000349 * 2.5;  // J for 5x 3" 0.53 lb flywheels
        public static final double kTurretMoI = 0.104506595;

        public static final double kShooterGearing = 1.0 / 1;
        public static final double kTurretGearing = 41.66666666 / 1;
        public static final double kHoodGearing = 25.0 / 1;

        public static final int kPeakShooterVolts = 16;

        // ---- turret limits ----
        // The turret can rotate ±0.55 rotations from its home position.
        // Software limits prevent it from wrapping the wiring.
        public static final Angle kTurretMaxRotsFromHome = Rotations.of(0.55);
        public static final Angle kTurretMinRots = Rotations.of(-kTurretMaxRotsFromHome.in(Rotations));
        public static final Angle kTurretMaxRots = Rotations.of(kTurretMaxRotsFromHome.in(Rotations));
        public static final double kTurretMaxErrD = Rotations.of(0.05).in(Rotations);
        public static final double kTurretMaxErrDSpin = Rotations.of(0.4).in(Rotations);

        // Range where the turret can't pass through (cable wrap / mechanical blockage).
        public static final double kTurretMaxNotAbleToPassRange = 0.23;
        public static final double kTurretMinNotAbleToPassRange = -0.23;

        // ---- shooter RPS targets ----
        public static final AngularVelocity kShooterMaxRPS = MotorK.kX44MaxVelocity.div(kShooterGearing);
        public static final double kShooterMaxRPSd = kShooterMaxRPS.in(RotationsPerSecond);
        public static final AngularVelocity kShooterRPS = kShooterMaxRPS.times(0.65);
        public static final double kShooterRPSd = 42.90 + 1.25;
        public static final AngularVelocity kShooterAutonCloseRPS = kShooterMaxRPS.times(0.60);  // auton is closer to hub
        public static final AngularVelocity kShooterAuton_EndSweep_RPS = kShooterMaxRPS.times(0.70); // end of sweep paths
        public static final AngularVelocity kShooterBarfRPS = kShooterMaxRPS.times(0.37);
        public static final AngularVelocity kShooterZeroRPS = RotationsPerSecond.zero();

        public static final AngularVelocity kShooterSpunUpMinimum = RotationsPerSecond.of(10);
        public static final Time kShooterSpunUpTimeout = Seconds.of(0.64);  // 2x expected spinup time as a safety margin
        public static final double kShooterSpunUpMinimumD = kShooterSpunUpMinimum.in(RotationsPerSecond);

        public static final double kDriverRPSIncreaseD = 2.0;

        // ---- hood constants ----
        public static final double kHoodMoI = 0.00027505;

        public static final Angle kHoodAbsoluteMinRots = Rotations.of(0.0);
        private static final Angle kHoodAbsoluteMaxRots = Rotations.of(1.174805);
        public static final Angle kHoodMaxDegs = Degrees.of(kHoodAbsoluteMaxRots.in(Degrees));
        public static final Angle kHoodLockDegs = Degrees.of(kHoodMaxDegs.times(0.75).in(Degrees));
        public static final double kHoodRotsd = 0.08;
        public static final double kHoodRotsHalfwayD = kHoodAbsoluteMaxRots.magnitude() * 0.75;
        public static final double kHoodEmergencyRotsD = Rotations.of(0.371338).magnitude();
        public static final double kHoodMaxErrD = Rotations.of(0.01).in(Rotations);
        public static final double kHoodAtPosTimeout = 0.1;

        // double versions for use in hot-path calculations
        public static final double kHoodMinRots_double = 0.0;
        public static final double kHoodMaxRots_double = kHoodAbsoluteMaxRots.in(Rotations);
        public static final double kPhysicalHoodMinPosition_double = 0;
        public static final double kPhysicalHoodMaxPosition_double = 48;

        public static final Angle kHoodTrenchPosition = Degrees.of(5);

        // Custom DC motor model for the hood (NEO 550 with specific characteristics).
        public static final DCMotor khoodDCMotorGearbox = new DCMotor(
            6,      // nominal voltage
            0.047,  // stall torque (N·m)
            2.5,    // stall current (A)
            0.2,    // free current (A)
            24.0855,// free speed (rad/s)
            1       // number of motors
        );

        // ---- hood homing ----
        // The hood also uses current-sense homing (same principle as the intake arm).
        public static final Current kWireTugMinAmps = Amps.of(8);
        public static final double kWireTugMinSecs = 0.125;
        public static final double kHoodHomingVoltage = -0.75;
        public static final Angle kHomingRetryReturnRots = Rotations.of(0.2);
        public static final Angle kHomePosition = Rotations.of(-0.2175);
        public static final Angle kInitPosition = Rotations.of(-0.145);

        // ---- CAN IDs ----
        public static final int kShooterA_CANID = 21;
        public static final int kShooterB_CANID = 20;
        public static final int kTurretCANID = 12;
        public static final int kHoodCANID = 22;

        // ---- TalonFX configurations ----
        // Slot 0: main shooting PID (used during normal operation)
        // Slot 1: alternate gains (used in specific scenarios, e.g. high-speed passing)
        private static final Slot0Configs kShooterASlot0Configs = new Slot0Configs()
            .withKS(0.37)   // static friction: minimum voltage to overcome stiction
            .withKV(0.1)    // velocity FF: volts per RPS
            .withKA(0)
            .withKP(0.5)    // proportional: corrects speed error
            .withKI(0)
            .withKD(0);
        private static final CurrentLimitsConfigs kShooterACurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true);
        private static final MotorOutputConfigs kShooterAOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        private static final FeedbackConfigs kShooterAFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kShooterGearing);
        private static final VoltageConfigs kShooterAVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(kPeakShooterVolts)
            .withPeakReverseVoltage(-kPeakShooterVolts);
        private static final Slot1Configs kShooterASlot1Configs = new Slot1Configs()
            .withKP(5)
            .withKI(0)
            .withKD(0)
            .withKS(4.5)
            .withKV(0.16)
            .withKA(0);
        public static final TalonFXConfiguration kShooterATalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kShooterASlot0Configs)
            .withSlot1(kShooterASlot1Configs)
            .withCurrentLimits(kShooterACurrentLimitConfigs)
            .withMotorOutput(kShooterAOutputConfigs)
            .withFeedback(kShooterAFeedbackConfigs)
            .withVoltage(kShooterAVoltageConfigs);

        // Shooter B uses the same config as A but with a different inversion.
        private static final MotorOutputConfigs kShooterBOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        public static final TalonFXConfiguration kShooterBTalonFXConfiguration = kShooterATalonFXConfiguration.clone()
            .withMotorOutput(kShooterBOutputConfigs);

        // ---- hood TalonFXS configuration ----
        // The hood uses a TalonFXS (for NEO 550 compatibility) instead of a TalonFX.
        private static final Slot0Configs kHoodSlot0Configs = new Slot0Configs()
            .withKP(29)
            .withKI(0)
            .withKD(0)
            .withKS(0.5)
            .withKV(4)
            .withKA(0)
            .withKG(0);  // gravity FF (not needed here since hood axis is horizontal)
        private static final CurrentLimitsConfigs kHoodCurrentLimitConfig = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(30)
            .withSupplyCurrentLimit(15)
            .withSupplyCurrentLowerLimit(5)
            .withSupplyCurrentLowerTime(1)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kHoodOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        private static final VoltageConfigs kHoodVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(16)
            .withPeakReverseVoltage(-16);
        private static final CommutationConfigs kHoodCommutationConfigs = new CommutationConfigs()
            .withAdvancedHallSupport(AdvancedHallSupportValue.Enabled)
            .withMotorArrangement(MotorArrangementValue.NEO550_JST);
        private static final ExternalFeedbackConfigs kHoodFeedbackConfigs = new ExternalFeedbackConfigs()
            .withSensorToMechanismRatio(kHoodGearing);

        // Software limits prevent the hood from driving past its physical travel range.
        public static final SoftwareLimitSwitchConfigs kHoodSoftLimitConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(kHoodAbsoluteMaxRots.minus(Rotations.of(0.05)))
            .withReverseSoftLimitThreshold(kHoodAbsoluteMinRots.plus(Rotations.of(0.05)))
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true);
        // Version with limits disabled — used during homing so the hood can reach its hard stop.
        public static final SoftwareLimitSwitchConfigs kHoodSoftLimitConfigsNoEnable = kHoodSoftLimitConfigs
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false);
        public static final TalonFXSConfiguration kHoodTalonFXSConfiguration = new TalonFXSConfiguration()
            .withSlot0(kHoodSlot0Configs)
            .withCurrentLimits(kHoodCurrentLimitConfig)
            .withMotorOutput(kHoodOutputConfigs)
            .withExternalFeedback(kHoodFeedbackConfigs)
            .withVoltage(kHoodVoltageConfigs)
            .withCommutation(kHoodCommutationConfigs)
            .withSoftwareLimitSwitch(kHoodSoftLimitConfigs);
        public static final TalonFXSConfiguration kHoodTalonFXSConfigurationNoSoftLimit = new TalonFXSConfiguration()
            .withSlot0(kHoodSlot0Configs)
            .withCurrentLimits(kHoodCurrentLimitConfig)
            .withMotorOutput(kHoodOutputConfigs)
            .withExternalFeedback(kHoodFeedbackConfigs)
            .withVoltage(kHoodVoltageConfigs)
            .withCommutation(kHoodCommutationConfigs)
            .withSoftwareLimitSwitch(kHoodSoftLimitConfigsNoEnable);

        // ---- turret TalonFX configuration ----
        private static final Slot0Configs kTurretSlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(5)
            .withKA(0.02)
            .withKP(300)
            .withKI(0)
            .withKD(5);
        private static final CurrentLimitsConfigs kTurretCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(55)
            .withSupplyCurrentLimit(55)
            .withSupplyCurrentLowerLimit(15)
            .withSupplyCurrentLowerTime(1.0) // drop to 15A after 1 second of high current
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kTurretOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        private static final MotionMagicConfigs kTurretMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(110)  //TODO: update cruise velocity after re-characterization
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

        // Turret absolute encoder (CANcoder) configuration.
        public static final MagnetSensorConfigs kEncoderAMagnetSensorConfigs = new MagnetSensorConfigs()
            .withMagnetOffset(TurretK.kEncAMagnetOffset);
        public static final CANcoderConfiguration kEncoderAConfiguration = new CANcoderConfiguration()
            .withMagnetSensor(kEncoderAMagnetSensorConfigs);

        // Fixed shooter pose overrides — used when auton shooting from known positions.
        // [Left, Center (Climb), Center (Hub), Right] from driver POV.
        public static final Pose2d kShooterOverridePose[] = {
            AllianceFlipUtil.apply(new Pose2d(FieldK.kFieldLengthMeters / 6, FieldK.kFieldWidthMeters * 2 / 3, new Rotation2d(0))),
            AllianceFlipUtil.apply(new Pose2d(Units.inchesToMeters(156.61 - 115.05 + 10), FieldK.kFieldWidthMeters / 2, new Rotation2d(0))),
            AllianceFlipUtil.apply(new Pose2d(Units.inchesToMeters(156.61 - 10), FieldK.kFieldWidthMeters / 2, new Rotation2d(0))),
            AllianceFlipUtil.apply(new Pose2d(FieldK.kFieldLengthMeters / 6, FieldK.kFieldWidthMeters / 3, new Rotation2d(0))),
        };
    }


    // =============================================================
    // VISION CONSTANTS
    // Camera positions relative to the robot center (used by PhotonVision).
    // Coordinates follow: ONSHAPE X = OUR Y, ONSHAPE Y = OUR X — don't mix these up.
    // =============================================================
    public static class VisionK {
        public static final Transform3d kFrontLeftCTR  = VisionUtil.transformToRobo(8.875,   12.18175,  20.45,   180, -20,  45);
        public static final Transform3d kFrontRightCTR = VisionUtil.transformToRobo(8.875,  -12.18175,  20.45,   180, -20, -45);
        public static final Transform3d kBackLeftCTR   = VisionUtil.transformToRobo(-11.375,  11.875,   20.5625,   0, -20, 135);
        public static final Transform3d kBackRightCTR  = VisionUtil.transformToRobo(-12.455, -12.055,   18.25,   180, -20, -135);
    }


    // =============================================================
    // FIELD CONSTANTS
    // Field dimensions and the April Tag layout used for vision pose estimation.
    // =============================================================
    public static class FieldK {
        // Field dimensions pulled from the 2026 Rebuilt field spec (welded version).
        public static final double kFieldLengthMeters = Units.inchesToMeters(651.22);
        public static final double kFieldWidthMeters = Units.inchesToMeters(317.69);

        // Reset poses for each alliance side (used when pressing the reset-pose button).
        public static final Pose2d kLeftResetPose  = new Pose2d(0.478, 8 - 0.392, Rotation2d.kZero);
        public static final Pose2d kRightResetPose = new Pose2d(0.478, 0.392,     Rotation2d.kZero);

        // April Tag layout with trench tags excluded. We remove them because the trench
        // area has poor camera coverage and including those tags hurts pose accuracy.
        public static final AprilTagFieldLayout kTagLayout;
        static {
            HashSet<Integer> excludedAprilTagsID = new HashSet<>(Arrays.asList(1, 6, 7, 12, 17, 22, 23, 28));
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            List<AprilTag> tags = new ArrayList<>(fieldLayout.getTags());
            tags.removeIf(tag -> excludedAprilTagsID.contains(tag.ID));
            kTagLayout = new AprilTagFieldLayout(tags, fieldLayout.getFieldLength(), fieldLayout.getFieldWidth());
        }
    }


    // =============================================================
    // ROBOT CONSTANTS
    // Physical dimensions and misc settings.
    // =============================================================
    public static class RobotK {
        public static final String kLogTab = "Robot";

        public static final int kMiniPCChannel = 14;

        public static final Distance kRobotFullWidth  = Inches.of(33.6875);
        public static final Distance kRobotFullLength = Inches.of(32.6875);
        public static final Distance kBumperHeight    = Inches.of(4.5);

        // Max drive speed while actively intaking (keeps the ball from bouncing off).
        public static final double kRobotSpeedIntakingLimit = 0.31;
        // Max drive speed while evading defense.
        public static final double kRobotEvasionLimit = 1.5;
    }


    // =============================================================
    // SUPERSTRUCTURE CONSTANTS
    // =============================================================
    public static class SuperstructureK {
        public static final String kLogTab = "Superstructure";
    }


    // =============================================================
    // INTAKE CONSTANTS
    // Arm and roller motor settings + TalonFX configurations.
    // =============================================================
    public static class IntakeK {
        public static final String kLogTab = "Intake";

        // ---- physical constants (used for simulation) ----
        public static final double kIntakeArmMOI      = 0.0209;       // moment of inertia (kg·m²)
        public static final double kIntakeArmGearing  = 125.0 / 1;    // 125:1 reduction

        public static final double kIntakeRollersMOI      = 0.0001;
        public static final double kIntakeRollersGearing  = 12.0 / 30; // 0.4:1 (rollers spin faster than motor)

        // ---- roller speed targets ----
        public static final AngularVelocity kIntakeRollersMaxRPS     = MotorK.kX60FOCMaxVelocity.div(kIntakeRollersGearing);
        public static final AngularVelocity kIntakeRollersShootRPS   = kIntakeRollersMaxRPS.times(0.2);
        public static final AngularVelocity kIntakeRollersShimmyRPS  = kIntakeRollersMaxRPS.times(0.2);
        public static final double kIntakeRollersBarfVolts   = -12;
        public static final double kIntakeRollersIntakeVolts = 11;
        public static final double kIntakeRollersShimmyVolts = 5;

        // ---- CAN IDs ----
        public static final int kIntakeArmCANID       = 40;
        public static final int kIntakeRollersA_CANID = 41;
        public static final int kIntakeRollersB_CANID = 42;

        // ---- TalonFX configurations ----

        // Intake arm: MotionMagic position control with soft current limiting.
        // Low current limits because the arm is geared heavily and doesn't need much torque.
        private static final CurrentLimitsConfigs kIntakeArmCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final Slot0Configs kIntakeArmSlot0Configs = new Slot0Configs()
            .withKS(1.5)  // high static friction — arm is heavy and geared
            .withKV(0)
            .withKA(0)
            .withKP(50)
            .withKI(0)
            .withKD(0);
        public static final MotorOutputConfigs kIntakeArmMotorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        private static final MotionMagicConfigs kIntakeArmMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(64)
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

        // Intake rollers A: velocity control. Higher current limits since rollers
        // need to grab and accelerate balls quickly.
        private static final CurrentLimitsConfigs kIntakeRollersACurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(55)
            .withSupplyCurrentLimit(35)
            .withSupplyCurrentLowerTime(0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final Slot0Configs kIntakeRollersASlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(0.048) // kV tuned to match voltage-to-velocity relationship
            .withKA(0)
            .withKP(0.05)
            .withKI(0)
            .withKD(0);
        public static final MotorOutputConfigs kIntakeRollersAMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        public static final FeedbackConfigs kIntakeRollersAFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kIntakeRollersGearing);
        private static final VoltageConfigs kIntakeRollersAVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12);
        public static final TalonFXConfiguration kIntakeRollersAConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kIntakeRollersACurrentLimitConfigs)
            .withSlot0(kIntakeRollersASlot0Configs)
            .withMotorOutput(kIntakeRollersAMotorOutputConfigs)
            .withFeedback(kIntakeRollersAFeedbackConfigs)
            .withVoltage(kIntakeRollersAVoltageConfigs);

        // Roller B mirrors A but with opposite inversion (they face each other on the robot).
        public static final MotorOutputConfigs kIntakeRollersBMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        public static final TalonFXConfiguration kIntakeRollersBConfiguration = kIntakeRollersAConfiguration.clone()
            .withMotorOutput(kIntakeRollersBMotorOutputConfigs);
    }


    // =============================================================
    // INDEXER CONSTANTS
    // Spindexer + tunnel motor settings and speed ratio math.
    // =============================================================
    public static class IndexerK {
        public static final String kLogTab = "Indexer";

        // ---- CAN IDs ----
        //TODO: verify IDs match physical robot wiring
        public static final int kSpindexerCANID = 10;
        public static final int kTunnelCANID    = 11;

        // ---- gear reductions ----
        public static final double kSpindexerGearing = 5.0;        // 5:1
        public static final double kTunnelGearing    = 20.0 / 18.0; // ~1.11:1

        // ---- moments of inertia (for simulation) ----
        public static final double kSpindexerMOI = 0.00166190059;
        public static final double kTunnelMOI    = 0.000215968064;

        // ---- spindexer speed targets ----
        public static final AngularVelocity kSpindexerMaxRPS   = MotorK.kX60MaxVelocity.div(kSpindexerGearing);
        public static final AngularVelocity kSpindexerIntakeRPS = kSpindexerMaxRPS.times(-0.10); // negative = reverse for intake
        public static final AngularVelocity kSpindexerShootRPS = kSpindexerMaxRPS.times(0.85);
        public static final double kSpindexerMaxRPSD   = kSpindexerMaxRPS.in(RotationsPerSecond);
        public static final double kSpindexerShootRPSD = kSpindexerShootRPS.in(RotationsPerSecond);
        public static final double kSpindexerIntakeRPSD = kSpindexerIntakeRPS.in(RotationsPerSecond);

        // ---- tunnel speed targets ----
        public static final AngularVelocity kTunnelMaxRPS    = MotorK.kX60FOCMaxVelocity.div(kTunnelGearing);
        public static final AngularVelocity kTunnelShootRPS  = kTunnelMaxRPS.times(0.77);
        public static final double kTunnelMaxRPSD   = kTunnelMaxRPS.in(RotationsPerSecond);
        public static final double kTunnelShootRPSD = kTunnelShootRPS.in(RotationsPerSecond);

        public static final AngularVelocity kTunnelSpunUpMinimum = RotationsPerSecond.of(10);
        public static final double kTunnelSpunUpMinimumD = 10.0;
        public static final Time kTunnelSpunUpTimeout = Seconds.of(1);

        // ---- speed ratio constants ----
        // These physical wheel radii are used to derive indexer speed from shooter speed.
        // The goal: match surface speeds at every ball handoff point so the ball flows
        // smoothly without getting slowed or jerked.
        //
        // Derivation (shooter → tunnel):
        //   surface speed = radius × angular velocity
        //   r_shooter × ω_shooter = r_tunnel_pulley × ω_tunnel
        //   → ratio = (r_bigFlywheel + r_smallFlywheel) / (2 × r_tunnelPulley)
        //   (average of the two flywheel radii since the ball contacts both)
        public static final double kR_bigFlywheel    = ShooterK.kFlywheelRadiusM; // 0.0381 m
        public static final double kR_smallFlywheel  = 0.0215;   // m
        public static final double kR_tunnelPulley   = 0.018;    // m
        public static final double kR_spindexerFloor = 6.5 * 0.0254; // 0.1651 m

        public static final double kTunnelFromShooterRatio    = (kR_bigFlywheel + kR_smallFlywheel) / (2.0 * kR_tunnelPulley);
        public static final double kSpindexerFromShooterRatio = (kR_bigFlywheel + kR_smallFlywheel) / (2.0 * kR_spindexerFloor);

        // ---- TalonFX configurations ----

        // Spindexer: velocity control, coast on stop.
        private static final Slot0Configs kSpindexerSlot0Configs = new Slot0Configs()
            .withKS(0.420)
            .withKV(0.560)
            .withKA(0)
            .withKP(1.5)
            .withKI(0)
            .withKD(0);
        private static final CurrentLimitsConfigs kSpindexerCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(65)
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLowerTime(0)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kSpindexerMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        private static final FeedbackConfigs kSpindexerFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kSpindexerGearing);
        private static final VoltageConfigs kSpindexerVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(16)
            .withPeakReverseVoltage(-16);
        public static final TalonFXConfiguration kSpindexerTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kSpindexerSlot0Configs)
            .withCurrentLimits(kSpindexerCurrentLimitConfigs)
            .withMotorOutput(kSpindexerMotorOutputConfigs)
            .withFeedback(kSpindexerFeedbackConfigs)
            .withVoltage(kSpindexerVoltageConfigs);

        // Tunnel: velocity control with FOC, coast on stop.
        private static final Slot0Configs kTunnelSlot0Configs = new Slot0Configs()
            .withKS(0.2)
            .withKV(0.1337)
            .withKA(0)
            .withKP(0.37)
            .withKI(0)
            .withKD(0);
        private static final CurrentLimitsConfigs kTunnelCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLowerTime(0)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kTunnelMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        private static final FeedbackConfigs kTunnelFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kTunnelGearing);
        private static final VoltageConfigs kTunnelVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(16)
            .withPeakReverseVoltage(-16);
        public static final TalonFXConfiguration kTunnelTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kTunnelSlot0Configs)
            .withCurrentLimits(kTunnelCurrentLimitConfigs)
            .withMotorOutput(kTunnelMotorOutputConfigs)
            .withFeedback(kTunnelFeedbackConfigs)
            .withVoltage(kTunnelVoltageConfigs);
    }


    // =============================================================
    // TURRET CONSTANTS
    // Encoder offsets and gear tooth counts for the turret position tracking system.
    // =============================================================
    public static class TurretK {
        public static final String kLogTab = "Turret";

        // Gear tooth counts for the turret's LCM (Least Common Multiple) absolute position tracking.
        // The combination of gear tooth counts creates a unique pattern used to find absolute position.
        public static final double kGearZeroToothCount = 100;
        public static final double kGearOneToothCount  = 10;
        public static final double kGearTwoToothCount  = 19;

        // The turret's LCM reading when it's at the mechanical home position.
        // Measured empirically — log "turretLCMPos" and read the value when the turret is at home.
        public static final double kLCMAtHomeRots = 0.251;

        // CANcoder magnet offset: corrects for the encoder not being physically zero-aligned.
        public static final double kEncAMagnetOffset = 0.320556640625;

        // Secondary encoder offset: the reading of encoder B when encoder A reads zero.
        // Used as a cross-check for the LCM position calculation.
        public static final double kEncBOffset = 0.529614;
    }


    // =============================================================
    // AUTON CONSTANTS
    // Timeouts and Choreo trajectory file names.
    // =============================================================
    public static class AutonK {
        public static final String kLogTab = "Auton";

        // ---- reference poses ----
        // Used for auton starting position reset and neutral-zone aiming.
        public static final Pose2d kRightNeutralPose = new Pose2d(Meters.of(6.924767017364502),
            Meters.of(2.251265048980713), new Rotation2d(0));
        public static final Pose2d kRightDepotPose = new Pose2d(Meters.of(1.1576627492904663),
            Meters.of(5.958622932434082), new Rotation2d(Math.PI));
        public static final Pose2d kLeftNeutralPose = new Pose2d(Meters.of(6.924767017364502),
            Meters.of(5.437880039215088), new Rotation2d(0));

        // ---- timeouts ----
        // How long each auton action is allowed to take before giving up and moving on.
        // These are safety cutoffs — in ideal conditions the action ends earlier.
        public static final double kIntakeTimeout       = 7.5;
        public static final double kShootingTimeout     = 4;
        public static final double kSOTMTimeout         = 100; // effectively unlimited — SOTM doesn't block on shot confirmation
        public static final double kSweepShootingTimeout = 20;

        // Delay (seconds) between segments when following another robot.
        public static final double kFollowDelay = 2;

        // ---- trajectory file names ----
        // These strings are the file names of the Choreo trajectory JSON files
        // (without the .traj extension). They're used by WaltAdaptableAutonFactory
        // and AutonChooser to load paths.
        //
        // Naming convention:
        //   SIDE_cycle_description
        //   e.g. "RIGHT_one_jab" = right-side start, first cycle, jab path
        //   "LEFT_two_sweep"     = left-side start, second cycle, sweep path

        /* OLD PATHS (kept for reference / regression testing) */
        public static final String kRightOneJab      = "RIGHT_one_jab";
        public static final String kRightOneTrench   = "RIGHT_one_trench";
        public static final String kRightOneDefense  = "RIGHT_one_defense";
        public static final String kRightOneReverse  = "RIGHT_one_reverse";

        public static final String kRightTwoSotmDepot = "RIGHT_two_sotmDepot";
        public static final String kRightTwoDepot     = "RIGHT_two_depot";
        public static final String kRightTwoSweep     = "RIGHT_two_sweep";
        public static final String kRightTwoPassing   = "RIGHT_two_passing";
        public static final String kRightTwoJab       = "RIGHT_two_jab";
        public static final String kRightTwoReverse   = "RIGHT_two_reverse";

        public static final String kLeftOneJab      = "LEFT_one_jab";
        public static final String kLeftOneTrench   = "LEFT_one_trench";
        public static final String kLeftOneDefense  = "LEFT_one_defense";
        public static final String kLeftOneReverse  = "LEFT_one_reverse";

        public static final String kLeftTwoSotmDepot = "LEFT_two_sotmDepot";
        public static final String kLeftTwoDepot     = "LEFT_two_depot";
        public static final String kLeftTwoSweep     = "LEFT_two_sweep";
        public static final String kLeftTwoPassing   = "LEFT_two_passing";
        public static final String kLeftTwoJab       = "LEFT_two_jab";
        public static final String kLeftTwoReverse   = "LEFT_two_reverse";

        public static final String kRightOneCircle        = "RIGHT_one_circle";
        public static final String kLeftOneSweepAndDepot  = "LEFT_one_sweepAndDepot";
        public static final String kLeftThreeDepotToBump  = "LEFT_three_depotToBump";
        public static final String kRightThreeDepotToBump = "RIGHT_three_depotToBump";

        public static final String kRightStressTestLong    = "RIGHT_stress_test_long";
        public static final String kRightStressTestOverlap = "RIGHT_stress_test_overlap";

        /* NEW PATHS */
        public static final String kRightOneBumpReturn       = "RIGHT_one_bumpReturn";
        public static final String kRightOneBumpReturnFollow = "RIGHT_one_bumpReturnFollow";
        public static final String kLeftOneBumpReturn        = "LEFT_one_bumpReturn";
        public static final String kLeftOneBumpReturnFollow  = "LEFT_one_bumpReturnFollow";
        public static final String kRightTwoBumpReturn       = "RIGHT_two_bumpReturn";
        public static final String kLeftTwoBumpReturn        = "LEFT_two_bumpReturn";
        public static final String kRightTwoBumpToTrench     = "RIGHT_two_bumpToTrench";
        public static final String kLeftTwoBumpToTrench      = "LEFT_two_bumpToTrench";

        public static final String kRightOneTrenchReturn     = "RIGHT_one_trenchReturn";
        public static final String kLeftOneTrenchReturn      = "LEFT_one_trenchReturn";
        public static final String kRightTwoTrenchReturn     = "RIGHT_two_trenchReturn";
        public static final String kLeftTwoTrenchReturn      = "LEFT_two_trenchReturn";
        public static final String kRightOneBumpTrenchReturn = "RIGHT_one_bumpReverseToTrench";
        public static final String kLeftOneBumpTrenchReturn  = "LEFT_one_bumpReverseToTrench";

        public static final String kRightOneTrenchToOutpost  = "RIGHT_one_trenchToOutpost";
        public static final String kRightTwoTrenchToOutpost  = "RIGHT_two_trenchToOutpost";
        public static final String kRightTwoOutpostToTrench  = "RIGHT_two_outpostToTrench";
        public static final String kRightOneBumpToOutpost    = "RIGHT_one_bumpToOutpost";
        public static final String kRightTwoBumpToOutpost    = "RIGHT_two_bumpToOutpost";
        public static final String kRightTwoOutpostToBump    = "RIGHT_two_outpostToBump";

        public static final String kLeftOneTrenchToDepot  = "LEFT_one_trenchToDepot";
        public static final String kLeftTwoTrenchToDepot  = "LEFT_two_trenchToDepot";
        public static final String kLeftTwoDepotToTrench  = "LEFT_two_depotToTrench";
        public static final String kLeftOneBumpToDepot    = "LEFT_one_bumpToDepot";
        public static final String kLeftTwoBumpToDepot    = "LEFT_two_bumpToDepot";
        public static final String kLeftTwoDepotToBump    = "LEFT_two_depotToBump";

        public static final String kRightOneSelfPass  = "RIGHT_one_selfPass";
        public static final String kLeftOneSelfPass   = "LEFT_one_selfPass";
        public static final String kRightTwoGoOut     = "RIGHT_two_goOut";
        public static final String kLeftTwoGoOut      = "LEFT_two_goOut";
        public static final String kRightBumpPreload  = "RIGHT_one_bumpPreload";
        public static final String kLeftBumpPreload   = "LEFT_one_bumpPreload";
        public static final String kRightTrenchPreload = "RIGHT_one_trenchPreload";
        public static final String kLeftTrenchPreload  = "LEFT_one_trenchPreload";

        public static final String kCenterPreload = "CENTER_one_preload";
    }
}
