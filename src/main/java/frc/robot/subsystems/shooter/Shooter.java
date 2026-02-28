package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
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

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.RobotK.kRobotFullWidth;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShotCalculator.ShotData;
import frc.util.AllianceFlipUtil;
import frc.util.GobildaServoAngled;
import frc.util.WaltMotorSim;
import frc.util.WaltTuner;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

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
    private final VoltageOut m_VoltageReq = new VoltageOut(0);
    private final StaticBrake m_BrakeReq = new StaticBrake();

    private final GobildaServoAngled m_hood = new GobildaServoAngled(kHoodChannel);
    private final CANcoder m_hoodEncoder = new CANcoder(kHoodEncoderCANID, Constants.kCanivoreBus);

    private Boolean m_isTurretCoast = false;
    private GenericEntry nte_turretCoast = WaltTuner.createBoolToggleSwitch(kLogTab, "TurretCoast", m_isTurretCoast);

    private DigitalInput m_turretHomingHall = new DigitalInput(2);
    private final BooleanLogger log_turretHomingHall = new BooleanLogger(kLogTab, "turretHomeHall");


    private Angle m_calcHood = Degrees.of(0);
    private Angle m_calcTurret = Rotations.of(0);
    private AngularVelocity m_calcFlywheel = RotationsPerSecond.of(0);

    //---LOGIC BOOLEANS
    private boolean m_isTurretHomed = false;
    //---TRIGGERS
    private final Trigger trg_inAllianceZone = new Trigger(this::inAllianceZone);
    private final Trigger trg_turretHomingCompleted = new Trigger(() -> m_isTurretHomed);

    /* SIM OBJECTS */
    private final FlywheelSim m_shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(2), kShooterMoI, kShooterGearing), DCMotor.getKrakenX60Foc(2) // returns gearbox
    );

    private final DCMotorSim m_turretSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1), kTurretMoI, kTurretGearing), DCMotor.getKrakenX44Foc(1) // returns gearbox
    );

    /* LOGGERS */
    private final DoubleLogger log_shooterVelocityRPS = WaltLogger.logDouble(kLogTab, "shooterVelocityRPS");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble(kLogTab, "turretPositionRots");

    //---HOOD
    private final DoubleLogger log_hoodEncoderPositionDegs = WaltLogger.logDouble(kLogTab, "hoodEncoderPositionDegs");
    private final DoubleLogger log_hoodEncoderVelocityRPS = WaltLogger.logDouble(kLogTab, "hoodEncoderVelocityRPS");
    private final DoubleLogger log_hoodEncoderError = WaltLogger.logDouble(kLogTab, "hoodEncoderError");
    private final DoubleLogger log_requestedServoPositionDegs = WaltLogger.logDouble(kLogTab, "requestedServoPositionDegs");
    private final DoubleLogger log_requestedHoodPositionDegs = WaltLogger.logDouble(kLogTab, "requestedHoodPositionDegs");

    // private final BooleanLogger log_exitBeamBreak = WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    private final DoubleLogger log_hoodServoVoltage = WaltLogger.logDouble(kLogTab, "hoodServoVoltage");
    private final DoubleLogger log_hoodServoCurrent = WaltLogger.logDouble(kLogTab, "hoodServoCurrent");
    private final DoubleLogger log_shooterClosedLoopError = WaltLogger.logDouble(kLogTab, "shooterClosedLoopError");

    /* CONSTRUCTOR */
    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        m_shooterLeader.getConfigurator().apply(kShooterLeaderTalonFXConfiguration);
        m_shooterFollower.getConfigurator().apply(kShooterFollowerTalonFXConfiguration);
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);
        m_hoodEncoder.getConfigurator().apply(kHoodEncoderConfiguration); // if needed, we can add a position offset

        m_shooterFollower.setControl(new Follower(kLeaderCANID, MotorAlignmentValue.Opposed)); 

        m_turretTurnPosition = m_turret.getPosition().getValue();

        m_flywheelVelocity = m_shooterLeader.getVelocity().getValue();

        m_poseSupplier = poseSupplier;
        m_fieldSpeedsSupplier = fieldSpeedsSupplier;

        m_isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

        m_turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(m_poseSupplier.get().rotateAround(
                        poseSupplier.get().getTranslation(), new Rotation2d(m_turretTurnPosition)))
                                .transformBy(kTurretTransform),
                fieldSpeedsSupplier);

        m_fuelSim = FuelSim.getInstance();

        setDefaultCommand(turretHomingCmd());

        trg_turretHomingCompleted.onTrue(Commands.runOnce(() -> setGoal(ShooterGoal.SCORING)));
        // if (inAllianceZone()) {
        //     setGoal(ShooterGoal.SCORING);
        // } else {
        //     setGoal(ShooterGoal.PASSING);
        // }

        m_turret.setPosition(0);

        initSim();
    }

    /* COMMANDS */
    public void zeroTurret() {
        setTurretPos(Rotations.of(0));
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
        setShooterVelocity(RotationsPerSecond.of(0.0));
    }

    public Command zeroFlywheelCmd() {
        return runOnce(this::stopFlywheel).ignoringDisable(true);
    }

    public Command zeroShooterCmd() {
        return Commands.sequence(zeroFlywheelCmd(), zeroHoodCmd(), zeroTurretCmd());
    }

    //---SHOOTER (Velocity Control)
    public Command setShooterVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> setShooterVelocity(RPS));
    }

    public void setShooterVelocity(AngularVelocity RPS) {
        m_shooterLeader.setControl(m_velocityRequest.withVelocity(RPS));
    }

    //for TestingDashboard
    public Command setShooterVelocityCmd(DoubleSubscriber sub_RPS) {
        return run(() -> setShooterVelocity(RotationsPerSecond.of(sub_RPS.get())));
    }

    public boolean isShooterSpunUp() {
        var err = m_shooterLeader.getClosedLoopError();
        log_shooterClosedLoopError.accept(err.getValueAsDouble());
        boolean isNear = m_shooterLeader.getClosedLoopError().isNear(0, 3);
        log_spunUp.accept(isNear);
        return isNear;
    }

    //---HOOD (Basic Position Control)
    public Command setHoodPositionCmd(Angle degs) {
        return runOnce(() -> setHoodPosition(degs));
    }

    public void setHoodPosition(Angle degs) {
        m_hood.setAngle(convertHoodAngleToServoAngle(degs));
    }

    public double convertHoodAngleToServoAngle(Angle hoodAngleDegs) {
        return (1 - (hoodAngleDegs.magnitude() / kHoodAbsoluteMaxDegs.magnitude())) * kHoodServoMaxDegs.magnitude();
    }

    public double convertServoAngleToHoodAngle(Angle servoAngleDegs) {
        return (1 - (servoAngleDegs.magnitude() / kHoodServoMaxDegs.magnitude())) * kHoodAbsoluteMaxDegs.magnitude();
    }

    public double convertEncoderAngleToServoAngle(Angle encoderAngleDegs) {
        return (1 - (encoderAngleDegs.magnitude() / kHoodEncoderMaxDegs.magnitude())) * kHoodServoMaxDegs.magnitude();
    }

    //for TestingDashboard
    public Command setHoodPositionCmd(DoubleSubscriber sub_degs) {
        return run(() -> setHoodPosition(Degrees.of(sub_degs.getAsDouble())));
    }

    // ---TURRET (Motionmagic Angle Control)
    public Command setTurretPositionCmd(Angle rots) {
        return runOnce(() -> setTurretPos(rots));
    }

    private void setTurretPos(Angle rots) {
        m_turret.setControl(m_MMVRequest.withPosition(rots));
    }

    // for TestingDashboard
    public Command setTurretPositionCmd(DoubleSubscriber sub_rots) {
        return run(() -> setTurretPos(Rotations.of(sub_rots.get())));
    }

    /* GETTERS */
    public AngularVelocity getShooterVelocity() {
        return m_shooterLeader.getVelocity().getValue();
    }

    public Angle getHoodAngle() {
        return m_hoodEncoder.getAbsolutePosition().getValue();
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

        AngularVelocity flywheelVelocity = getShooterVelocity();    //current flywheel velocity
        Angle hoodAngle = Degrees.of(90).minus(getHoodAngle()); //current hood angle (need to subtract from 90 to get an accurate launching angle)
        Angle turretPosition = getTurretPosition(); //current turret position
        LinearVelocity flywheelLinearVelocity = ShotCalculator
                .angularToLinearVelocity(flywheelVelocity, kFlywheelRadius);

        m_fuelSim.launchFuel(flywheelLinearVelocity, hoodAngle, turretPosition,
            kTurretTransform.getMeasureZ()); //launch from where the turret *should* be
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
     * Testing Method - Sets the current aiming position ahead of the robot distanceMeters ahead of the robot.
     * 
     * @param distanceMeters how far ahead (in X) of the robot the target will be 
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
     * @param robotPose current robotPose
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
     */
    public void setGoal(ShooterGoal goal) {
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
        //TODO: once we finish our lerp, switch to the iterativeMovingShotFromInterpolationMap method
        ShotData calculatedShot = ShotCalculator.iterativeMovingShotFromFunnelClearance(robotPose, fieldSpeeds,         //The Calculated shot itself, according to the current robotPose, robotSpeeds, and the currentTarget
                currentTarget, 3);                                                                               
        Angle azimuthAngle = ShotCalculator.calculateAzimuthAngle(robotPose, calculatedShot.getTarget(),                //The turret angle according to the Calculated shot
                m_turret.getPosition().getValue());
        setTurretPos(azimuthAngle);                                                                                     //Sets the TurretPosition to the Calculated TurretAngle
        setHoodPosition(calculatedShot.getHoodAngle());                                                                 //Sets the HoodPosition to the Calculated HoodAngle
        // setShooterVelocity(ShotCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(), kFlywheelRadius));  //Sets the ShooterVelocity to the Calculated ShooterVelocity
        
        m_calcTurret = azimuthAngle;
        m_calcHood = calculatedShot.getHoodAngle();
        m_calcFlywheel = ShotCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(), kFlywheelRadius);
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
        setTurretPos(azimuthAngle);

        m_calcTurret = azimuthAngle;
    }

    /**
     * Tells us whether or not our Turret, Hood, and Flywheel are at their desired position with a certain amount of tolerance relative to the object
     * 
     * Hood Tolerance: .5 degrees. Turret Tolerance: __ rotations. Flywheel Tolerance: __ RPS.
     * @return
     */
    public boolean atPosition() {
        var turretAtPos = getTurretPosition().isNear(m_calcTurret, Rotations.of(0)); //TODO: get the turret tolerance
        var flywheelAtPos = getShooterVelocity().isNear(m_calcFlywheel, RotationsPerSecond.of(0)); //TODO: get the flywheel tolerance
        var hoodAtPos = getHoodAngle().isNear(Degrees.of(convertHoodAngleToServoAngle(m_calcHood)), kHoodShootingTolerance); //is this right buh
        return turretAtPos && flywheelAtPos && hoodAtPos;
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        Pose2d pose = m_poseSupplier.get();

        // trg_inAllianceZone.and(DriverStation::isTeleop)
        //     .onTrue(Commands.runOnce(() -> setGoal(ShooterGoal.SCORING)))
        //     .onFalse(Commands.runOnce(() -> setGoal(ShooterGoal.PASSING)));
        // trg_inAllianceZone.negate().and(DriverStation::isTeleop).whileTrue(Commands.runOnce(() -> setGoal(ShooterGoal.PASSING)));

        switch (m_goal) {
            case SCORING:
                calculateShot(pose);
                break;
            case PASSING:
                calculateShot(pose);
                break;
            case TEST:
                calculateTurretAngle(pose);
                break;
            case OFF:
                break;
        }

        WaltTuner.toggleMotorCoast(m_isTurretCoast, nte_turretCoast.getBoolean(false), m_turret);

        //---Loggers
        m_turretTurnPosition = m_turret.getPosition().getValue();
        m_flywheelVelocity = m_shooterLeader.getVelocity().getValue();

        m_turretVisualizer.update3dPose(m_turretTurnPosition, getHoodAngle());

        log_shooterVelocityRPS.accept(m_shooterLeader.getVelocity().getValueAsDouble());
        log_hoodEncoderPositionDegs.accept(convertServoAngleToHoodAngle(Degrees.of(convertEncoderAngleToServoAngle(
                Degrees.of(Rotations.of(m_hoodEncoder.getAbsolutePosition().getValueAsDouble()).in(Degrees))))));
        log_hoodEncoderVelocityRPS.accept(m_hoodEncoder.getVelocity().getValueAsDouble());
        log_requestedServoPositionDegs.accept(m_hood.getAngle());
        log_requestedHoodPositionDegs.accept(convertServoAngleToHoodAngle(Degrees.of(m_hood.getAngle())));

        log_hoodEncoderError.accept(Math.abs((convertServoAngleToHoodAngle(Degrees.of(m_hood.getAngle())))
                - convertServoAngleToHoodAngle(Degrees.of(convertEncoderAngleToServoAngle(Degrees
                        .of(Rotations.of(m_hoodEncoder.getAbsolutePosition().getValueAsDouble()).in(Degrees)))))));

        log_turretPositionRots.accept(m_turret.getPosition().getValueAsDouble());

        // log_exitBeamBreak.accept(trg_exitBeamBreak);
        log_spunUp.accept(isShooterSpunUp());
        log_hoodServoVoltage.accept(RobotController.getVoltage6V());
        log_hoodServoCurrent.accept(RobotController.getCurrent6V());

        log_turretHomingHall.accept(m_turretHomingHall.get());
    }

    @Override
    public void simulationPeriodic() {
        m_turretVisualizer.updateFuel(
                ShotCalculator.angularToLinearVelocity(m_flywheelVelocity, kFlywheelRadius),
                getHoodAngle());

        WaltMotorSim.updateSimFX(m_shooterLeader, m_shooterSim);
        WaltMotorSim.updateSimFX(m_turret, m_turretSim);
        // WaltMotorSim.updateSimServo(m_hood, m_hoodSim);
    }

    public Command turretHomingCmd() {
        Runnable init = () -> {
            m_turret.setControl(m_VoltageReq.withOutput(-1.5));
        };

        Consumer<Boolean> end = (Boolean interrupted) -> {
            m_turret.setPosition(Rotations.of(-0.2175)); // Flowkirkentologicalexpialibrostatenuinely
            m_turret.setControl(m_BrakeReq);
            removeDefaultCommand();
            m_isTurretHomed = true;
            // setDefaultCommand(new PrintCommand("ShooterDefault"));
        };

        BooleanSupplier isFinished = () -> {
            return !m_turretHomingHall.get();
        };

        return new FunctionalCommand(init, () ->{}, end, isFinished, this);
            // .andThen(Commands.waitSeconds(0.1))
            // .andThen(runOnce(() -> {m_turret.setControl(m_MMVRequest.withPosition(0.1125));}));
    }

    /* CONSTANTS */
    public enum ShooterGoal {
        SCORING,
        PASSING,
        TEST,
        OFF
    }

}