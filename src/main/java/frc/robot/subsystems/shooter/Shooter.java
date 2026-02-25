package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.RobotK.kRobotFullWidth;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShotCalculator.ShotData;
import frc.util.AllianceFlipUtil;
import frc.util.WaltMotorSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.Pose2dLogger;

public class Shooter extends SubsystemBase {
    /* VARIABLES */
    private ShooterGoal m_goal = ShooterGoal.OFF;

    private AngularVelocity m_flywheelVelocity;
    private Angle m_turretTurnPosition;

    private int m_fuelStored = 8;

    private final Supplier<Pose2d> m_poseSupplier;
    private final Supplier<ChassisSpeeds> m_fieldSpeedsSupplier;

    private final Distance fieldWidthDiv2 = Inches.of(FieldConstants.fieldWidth / 2);

    private boolean m_isBlue;
    //need to implement the differing targets (if in neutral zone, shoot to X point (passing))
    Translation3d currentTarget = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);

    private final TurretVisualizer m_turretVisualizer;
    private final FuelSim m_fuelSim;

    // ---MOTORS + CONTROL REQUESTS
    private final TalonFX m_shooterLeader = new TalonFX(kLeaderCANID, Constants.kCanivoreBus); // X60Foc
    private final TalonFX m_shooterFollower = new TalonFX(kFollowerCANID, Constants.kCanivoreBus); // X60Foc
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private final TalonFX m_turret = new TalonFX(kTurretCANID, Constants.kCanivoreBus); // X44Foc
    private final MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    private final Servo m_hood = new Servo(kHoodChannel);
    private final Canandmag m_hoodEncoder = new Canandmag(kHoodEncoderChannel);
    private final PIDController m_hoodPID = new PIDController(1, 0, 0);

    private Angle m_hoodSetpoint = Degrees.of(0);
    private Angle m_currentHoodPos = Rotations.of(0);

    //---LOGIC BOOLEANS
    private boolean m_spunUp = false; // currently unused
    private final boolean m_inSim = RobotBase.isSimulation();

    //---LOGIC TRIGGERS
    private final Trigger trg_inAllianceZone = new Trigger(this::inAllianceZone);

    //---BEAM BREAKS (if we have one on the shooter)
    public DigitalInput m_exitBeamBreak = new DigitalInput(kExitBeamBreakChannel);
    public final Trigger trg_exitBeamBreak = new Trigger(() -> !m_exitBeamBreak.get()); //true when beam is broken

    public final Trigger exitBeamBreakTrigger(EventLoop loop) {
        return new Trigger(loop, () -> !m_exitBeamBreak.get());
    }

    /* SIM OBJECTS */
    private final FlywheelSim m_shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(2), kShooterMoI, kShooterGearing), DCMotor.getKrakenX60Foc(2) // returns gearbox
    );

    // Since the servo acts like a DC Motor, we use DCMotorSim
    private final DCMotorSim m_hoodSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(khoodDCMotorGearbox, kHoodMoI, kHoodGearing),
            khoodDCMotorGearbox // returns gearbox
    );

    private final DCMotorSim m_turretSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1), kTurretMoI, kTurretGearing), DCMotor.getKrakenX44Foc(1) // returns gearbox
    );

    /* LOGGERS */
    private final DoubleLogger log_shooterVelocityRPS = WaltLogger.logDouble(kLogTab,
            "shooterVelocityRPS");
    private final DoubleLogger log_hoodPositionDegs = WaltLogger.logDouble(kLogTab,
            "hoodPositionDegs");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble(kLogTab,
            "turretPositionRots");

    private final BooleanLogger log_exitBeamBreak = WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    private final Pose2dLogger log_calculateShotCurrPose = WaltLogger.logPose2d(kLogTab,
            "calculateShotCurrPose");

    private final BooleanLogger log_hoodEncoderMagnetInRange = WaltLogger.logBoolean(kLogTab,
            "hoodEncoderMagnetInRange");
    private final BooleanLogger log_hoodEncoderPresent = WaltLogger.logBoolean(kLogTab,
            "hoodEncoderPresent");

    private final DoubleLogger log_hoodPIDOutput = WaltLogger.logDouble(kLogTab, "hoodPIDOutput");

    /* CONSTRUCTOR */
    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        m_shooterLeader.getConfigurator().apply(kShooterLeaderTalonFXConfiguration);
        m_shooterFollower.getConfigurator().apply(kShooterFollowerTalonFXConfiguration);
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);

        m_hoodEncoder.setSettings(kHoodEncoderSettings); //if needed, we can add a position offset

        m_shooterFollower.setControl(new Follower(kLeaderCANID, MotorAlignmentValue.Opposed)); //TODO: check if MotorAlignmentValue is Opposed or Aligned

        m_turretTurnPosition = m_turret.getPosition().getValue();

        m_flywheelVelocity = m_shooterLeader.getVelocity().getValue();

        m_poseSupplier = poseSupplier;
        m_fieldSpeedsSupplier = fieldSpeedsSupplier;

        m_isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

        initSim();

        m_turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(m_poseSupplier.get().rotateAround(
                        poseSupplier.get().getTranslation(), new Rotation2d(m_turretTurnPosition)))
                                .transformBy(kTurretTransform),
                fieldSpeedsSupplier);

        m_fuelSim = FuelSim.getInstance();

        if (inAllianceZone()) {
            setDefaultCommand(setGoal(ShooterGoal.SCORING));
        } else {
            setDefaultCommand(setGoal(ShooterGoal.PASSING));
        }
    }

    /* COMMANDS */

    public void zeroTurret() {
        setTurretPosition(Rotations.of(0));
    }

    public Command zeroTurretCmd() {
        return runOnce(this::zeroTurret).ignoringDisable(true);
    }

    public void zeroHood() {
        setHoodPosition(Degrees.of(5));
    }

    public Command zeroHoodCmd() {
        return runOnce(this::zeroHood).ignoringDisable(true);
    }

    private void stopFlywheel() {
        m_shooterLeader.setNeutralMode(NeutralModeValue.Coast);
        setShooterVelocity(0.0);
    }

    public Command zeroFlywheelCmd() {
        return runOnce(this::stopFlywheel).ignoringDisable(true);
    }

    public Command zeroShooterCmd() {
        return Commands.sequence(zeroFlywheelCmd(), zeroHoodCmd(), zeroTurretCmd());
    }

    //---SHOOTER (Velocity Control)
    public Command setShooterVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> m_shooterLeader.setControl(m_velocityRequest.withVelocity(RPS)));
    }

    public void setShooterVelocity(AngularVelocity velocity) {
        m_shooterLeader.setControl(m_velocityRequest.withVelocity(velocity));
    }

    public void setShooterVelocity(Double velocity) {
        m_shooterLeader.setControl(m_velocityRequest.withVelocity(velocity));
    }

    //for TestingDashboard
    public Command setShooterVelocityCmd(DoubleSubscriber sub_RPS) {
        return run(() -> m_shooterLeader
                .setControl(m_velocityRequest.withVelocity(RotationsPerSecond.of(sub_RPS.get()))));
    }

    //---HOOD (Basic Position Control)
    public Command setHoodPositionCmd(Angle degs) {
        return runOnce(() -> m_hoodSetpoint = degs);
    }

    public void setHoodPosition(Angle degrees) {
        m_hoodSetpoint = degrees;
    }

    //for TestingDashboard
    public Command setHoodPositionCmd(DoubleSubscriber sub_degs) {
        return run(() -> m_hoodSetpoint = Degrees.of(sub_degs.get()));
    }

    // The PIDOutput needed to get to the setpoint from the current point
    public void updateHood() {
        // can't read from hardware in Sim, so we read from the hoodSim object
        m_currentHoodPos = m_inSim ? Rotations.of(m_hoodSim.getAngularPositionRotations())
                : Rotations.of(m_hoodEncoder.getPosition());

        double hoodPIDOutput = m_hoodPID.calculate(m_currentHoodPos.magnitude(),
                m_hoodSetpoint.in(Rotations)) * 8;

        hoodPIDOutput = MathUtil.clamp(hoodPIDOutput, -1.0, 1.0);
        hoodPIDOutput = (hoodPIDOutput + 1) / 2;
        m_hood.set(hoodPIDOutput);

        log_hoodPIDOutput.accept(hoodPIDOutput);
    }

    //---TURRET (Motionmagic Angle Control)
    public Command setTurretPositionCmd(Angle rots) {
        return runOnce(() -> m_turret.setControl(m_MMVRequest.withPosition(rots)));
    }

    public void setTurretPosition(Angle azimuthAngle) {
        m_turret.setControl(m_MMVRequest.withPosition(azimuthAngle));
    }

    //for TestingDashboard
    public Command setTurretPositionCmd(DoubleSubscriber sub_rots) {
        return run(
                () -> m_turret.setControl(m_MMVRequest.withPosition(Rotations.of(sub_rots.get()))));
    }

    /* GETTERS */
    public TalonFX getShooter() {
        return m_shooterLeader;
    }

    public AngularVelocity getShooterVelocity() {
        return m_shooterLeader.getVelocity().getValue();
    }

    public DCMotorSim getHoodSimEncoder() {
        return m_hoodSim;
    }

    public Angle getHoodAngle() {
        return m_currentHoodPos;
    }

    public TalonFX getTurret() {
        return m_turret;
    }

    public Angle getTurretPosition() {
        return m_turret.getPosition().getValue();
    }

    /* SIMULATION */
    public boolean simAbleToIntake() {
        return canIntake();
    }

    public void simIntake() {
        intakeFuel();
    }

    /**
     * @return true if robot can store more fuel
     */
    public boolean canIntake() {
        return m_fuelStored < kHopperCapacity;
    }

    public void intakeFuel() {
        m_fuelStored++;
    }

    /**
     * Launches SIMULATION FUEL™ at the current Flywheel Velocity, current Hood Angle, and the
     * current Turret Position.
     */
    public void launchFuel() {
        if (m_fuelStored == 0)
            return;
        // m_fuelStored--;

        AngularVelocity flywheelVelocity = getShooterVelocity();
        Angle hoodAngle = Degrees.of(90).minus(getHoodAngle());
        Angle turretPosition = getTurretPosition();
        LinearVelocity flywheelLinearVelocity = ShotCalculator
                .angularToLinearVelocity(flywheelVelocity, kFlywheelRadius);

        m_fuelSim.launchFuel(flywheelLinearVelocity, hoodAngle, turretPosition,
                kTurretTransform.getMeasureZ());
    }

    // TODO: update orientation values (if needed)
    private void initSim() {
        WaltMotorSim.initSimFX(m_shooterLeader, ChassisReference.CounterClockwise_Positive,
                TalonFXSimState.MotorType.KrakenX60);
        WaltMotorSim.initSimFX(m_turret, ChassisReference.CounterClockwise_Positive,
                TalonFXSimState.MotorType.KrakenX44);
    }

    /* ShootOnTheMove™ */
    /**
     * Sets the current aiming position to the target position, relative to what Alliance you are
     * on.
     * 
     * @param target desired target position.
     */
    public void setTarget(Translation3d target) {
        currentTarget = target;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            Translation2d flipped = AllianceFlipUtil.apply(target.toTranslation2d());
            currentTarget = new Translation3d(flipped.getX(), flipped.getY(), target.getZ());
        }
    }

    /**
     * Sets the current aiming position ahead of the robot distanceMeters ahead of the robot.
     * 
     * @param distanceMeters how far ahead of the robot the target will be
     */
    private void setTargetAheadOfRobot(double distanceMeters) {
        Pose2d currentPose = m_poseSupplier.get();

        Translation2d ahead = currentPose.getTranslation()
                .plus(new Translation2d(distanceMeters, currentPose.getRotation()));

        //set target at a standard height
        setTarget(new Translation3d(ahead.getX(), ahead.getY(), 2.0));
    }

    /**
     * Passing Method - Determines which passing target you will be shooting to based off of current
     * position on the field, and which alliance you are on.
     * 
     * @param robotPose curret robotPose
     * @return kPassingSpotLeft if the robot is on the left side of the field, kPassingSpotRight if
     *         the robot is on the right side of the field. Relative of what alliance one is on.
     */
    private Translation3d getPassingTarget(Pose2d robotPose) {
        boolean onLeftSide = m_isBlue ? robotPose.getMeasureY().gt(fieldWidthDiv2) : robotPose.getMeasureY().lt(fieldWidthDiv2);

        return onLeftSide ? kPassingSpotLeft : kPassingSpotRight;
    }

    /**
     * Sets the Goal State of the Shooter to either SCORING, PASSING, TEST, or OFF.
     * Method is incomplete for now, but add other methods that should be running passively when in certain goal states.
     * @param goal ShooterGoal desired
     * @return Command that runs the setTarget, setTargetAheadOfRobot, or zeroShooterCmd methods.
     */
    public Command setGoal(ShooterGoal goal) {
        return runOnce(() -> {
            m_goal = goal;
            switch (goal) {
            case SCORING:
                setTarget(FieldConstants.Hub.innerCenterPoint);
                removeDefaultCommand();
                break;
            case PASSING:
                setTarget(getPassingTarget(m_poseSupplier.get()));
                removeDefaultCommand();
                break;
            case TEST:
                setTargetAheadOfRobot(3);
                removeDefaultCommand();
                break;
            case OFF:
                zeroShooterCmd();
                removeDefaultCommand();
                break;
            }
        });
    }

    /**
     * Determines whether the robot is within the Alliance zone of the Alliance you are on.
     * 
     * @return true if the robot is in the AllianceZone, false otherwise.
     */
    private boolean inAllianceZone() {
        //UNTESTED
        Pose2d pose = m_poseSupplier.get();
       
        return (m_isBlue &&  Inches.of(pose.getMeasureX().in(Inches)).lt(Inches.of(Units.metersToInches(FieldConstants.LinesVertical.allianceZone)).plus(kRobotFullWidth.div(2)))) || //are we in the BLUE ALLIANCE ZONE as a BLUE ROBOT (behind the starting line effectively)
                    (!m_isBlue && Inches.of(pose.getMeasureX().in(Inches)).gt(Inches.of(Units.metersToInches(FieldConstants.LinesVertical.oppAllianceZone)).plus(kRobotFullWidth.div(2))));      //are we in the RED ALLIANCE ZONE as a RED ROBOT(behind the starting line effectively)
    }

    /**
     * Calculates the ideal shot to put the FUEL™ into the HUB™
     * 
     * @param robotPose current Robot position.
     */
    private void calculateShot(Pose2d robotPose) {
        ChassisSpeeds fieldSpeeds = m_fieldSpeedsSupplier.get();                                                        //How fast the robot is currently going, (CURRENT ROBOT VELOCITY)
        ShotData calculatedShot = ShotCalculator.iterativeMovingShotFromFunnelClearance(robotPose, fieldSpeeds,         //The Calculated shot itself, according to the current robotPose, robotSpeeds, and the currentTarget
                currentTarget, 3);                                                                               
        Angle azimuthAngle = ShotCalculator.calculateAzimuthAngle(robotPose, calculatedShot.getTarget(),                //The turret angle according to the Calculated shot
                m_turret.getPosition().getValue());
        setTurretPosition(azimuthAngle);                                                                                //Sets the TurretPosition to the Calculated TurretAngle
        setHoodPosition(calculatedShot.getHoodAngle());                                                                 //Sets the HoodPosition to the Calculated HoodAngle
        setShooterVelocity(ShotCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(), kFlywheelRadius));  //Sets the ShooterVelocity to the Calculated ShooterVelocity

    }

    /**
     * Version of calculateShot where, FOR TESTING, the turret will align to the target.
     * 
     * @param robotPose current Robot position
     */
    private void calculateTurretAngle(Pose2d robotPose) {
        ChassisSpeeds fieldSpeeds = m_fieldSpeedsSupplier.get();

        ShotData calculatedShot = ShotCalculator.iterativeMovingShotFromFunnelClearance(
                robotPose,
                fieldSpeeds,
                currentTarget,
                3);
        Angle azimuthAngle = ShotCalculator.calculateAzimuthAngle(
                robotPose,
                calculatedShot.getTarget(),
                m_turret.getPosition().getValue());
        setTurretPosition(azimuthAngle);
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        Pose2d pose = m_poseSupplier.get();

        log_calculateShotCurrPose.accept(pose);

        trg_inAllianceZone.and(DriverStation::isTeleop).whileTrue(setGoal(ShooterGoal.SCORING));
        trg_inAllianceZone.negate().and(DriverStation::isTeleop).whileTrue(setGoal(ShooterGoal.PASSING));

        switch (m_goal) {
            case SCORING:
                calculateShot(pose);
                break;
            case PASSING:
                calculateShot(pose);;
                break;
            case TEST:
                calculateTurretAngle(pose);
                break;
            case OFF:
                break;
        }

        //---Hood
        updateHood();

        //---Loggers
        log_shooterVelocityRPS.accept(m_shooterLeader.getVelocity().getValueAsDouble());
        log_hoodPositionDegs.accept(m_currentHoodPos.in(Degrees));
        log_turretPositionRots.accept(m_turret.getPosition().getValueAsDouble());

        log_exitBeamBreak.accept(trg_exitBeamBreak);
        log_spunUp.accept(m_spunUp);

        m_turretTurnPosition = m_turret.getPosition().getValue();
        m_flywheelVelocity = m_shooterLeader.getVelocity().getValue();

        m_turretVisualizer.update3dPose(m_turretTurnPosition, m_currentHoodPos);
        log_hoodEncoderMagnetInRange.accept(m_hoodEncoder.magnetInRange());
        log_hoodEncoderPresent.accept(m_hoodEncoder.isConnected());
    }

    @Override
    public void simulationPeriodic() {
        m_turretVisualizer.updateFuel(
                ShotCalculator.angularToLinearVelocity(m_flywheelVelocity, kFlywheelRadius),
                m_currentHoodPos);

        WaltMotorSim.updateSimFX(m_shooterLeader, m_shooterSim);
        WaltMotorSim.updateSimFX(m_turret, m_turretSim);
        WaltMotorSim.updateSimServo(m_hood, m_hoodSim);
    }

    /* CONSTANTS */
    public enum FlywheelVelocity {
        // in RPS
        ZERO(0),
        SCORE(5.5),
        PASS(7),
        MAX(20);

        public double RPS;

        private FlywheelVelocity(double RPS) {
            this.RPS = RPS;
        }
    }

    public enum HoodPosition {
        MIN(0),
        HOME(5),
        SCORE(10),
        PASS(20),
        MAX(40);

        public double rots;

        private HoodPosition(double rots) {
            this.rots = rots;
        }
    }

    public enum TurretPosition {
        MIN(0),
        HOME(10),
        SCORE(20),
        PASS(30),
        MAX(40);

        public double rots;

        private TurretPosition(double rots) {
            this.rots = rots;
        }
    }

    public enum ShooterGoal {
        SCORING,
        PASSING,
        TEST,
        OFF
    }

}