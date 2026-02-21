package frc.robot.subsystems;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
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
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.FuelSim;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.TurretVisualizer;
import frc.robot.subsystems.shooter.ShotCalculator.ShotData;
import frc.util.AllianceFlipUtil;
import frc.util.WaltMotorSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.Pose2dLogger;

public class Shooter extends SubsystemBase {
    /* VARIABLES */

    private TurretGoal m_goal = TurretGoal.OFF;

    private AngularVelocity m_flywheelVelocity;
    private Angle m_turretTurnPosition;

    private int m_fuelStored = 8;

    private final Supplier<Pose2d> m_poseSupplier;
    private final Supplier<ChassisSpeeds> m_fieldSpeedsSupplier;

    //need to implement the differing targets (if in neutral zone, shoot to X point (passing))
    Translation3d currentTarget = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);

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
    private boolean m_spunUp = false;   // currently unused
    private final boolean m_inSim = RobotBase.isSimulation();

    //---BEAM BREAKS (if we have one on the shooter)
    public DigitalInput m_exitBeamBreak = new DigitalInput(kExitBeamBreakChannel);
    public final Trigger trg_exitBeamBreak = new Trigger(() -> !m_exitBeamBreak.get()); //true when beam is broken

    public final Trigger exitBeamBreakTrigger(EventLoop loop) {
        return new Trigger(loop, () -> !m_exitBeamBreak.get());
    }

    /* SIM OBJECTS */
    private final FlywheelSim m_shooterSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(2), 
            kShooterMoI,
            kShooterGearing
        ),
        DCMotor.getKrakenX60Foc(2) // returns gearbox
    );

    // Since the servo acts like a DC Motor, we use DCMotorSim
    private final DCMotorSim m_hoodSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            khoodDCMotorGearbox,
            kHoodMoI,
            kHoodGearing
        ),
        khoodDCMotorGearbox // returns gearbox
    );

    private final DCMotorSim m_turretSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1),
            kTurretMoI,
            kTurretGearing
        ),
        DCMotor.getKrakenX44Foc(1) // returns gearbox
    );

    /* LOGGERS */
    private final DoubleLogger log_shooterVelocityRPS = WaltLogger.logDouble(kLogTab, "shooterVelocityRPS");
    private final DoubleLogger log_hoodPositionDegs = WaltLogger.logDouble(kLogTab, "hoodPositionDegs");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble(kLogTab, "turretPositionRots");

    private final BooleanLogger log_exitBeamBreak = WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");
    
    private final Pose2dLogger log_calculateShotCurrPose = WaltLogger.logPose2d(kLogTab, "calculateShotCurrPose");

    private final BooleanLogger log_hoodEncoderMagnetInRange = WaltLogger.logBoolean(kLogTab, "hoodEncoderMagnetInRange");
    private final BooleanLogger log_hoodEncoderPresent = WaltLogger.logBoolean(kLogTab, "hoodEncoderPresent");

    private final DoubleLogger log_hoodPIDOutput = WaltLogger.logDouble(kLogTab, "hoodPIDOutput");

    /* CONSTRUCTOR */
    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        m_shooterLeader.getConfigurator().apply(kShooterLeaderTalonFXConfiguration);
        m_shooterFollower.getConfigurator().apply(kShooterFollowerTalonFXConfiguration);
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);

        m_hoodEncoder.setSettings(kHoodEncoderSettings);    //if needed, we can add a position offset

        m_shooterFollower.setControl(new Follower(kLeaderCANID, MotorAlignmentValue.Opposed)); //TODO: check if MotorAlignmentValue is Opposed or Aligned

        m_turretTurnPosition = m_turret.getPosition().getValue();

        m_flywheelVelocity = m_shooterLeader.getVelocity().getValue();

        m_poseSupplier = poseSupplier;
        m_fieldSpeedsSupplier = fieldSpeedsSupplier;

        initSim();

        m_turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(m_poseSupplier
                        .get()
                        .rotateAround(poseSupplier.get().getTranslation(), new Rotation2d(m_turretTurnPosition)))
                        .transformBy(kRobotToTurret),
                fieldSpeedsSupplier);

        m_fuelSim = FuelSim.getInstance();
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
        return Commands.sequence(
                zeroFlywheelCmd(),
                zeroHoodCmd(),
                zeroTurretCmd());
    }

    //---SHOOTER (Veloity Control)
    public Command setShooterVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> m_shooterLeader.setControl(m_velocityRequest.withVelocity(RPS)));
    }

    public void setShooterVelocity(AngularVelocity velocity) {
        m_shooterLeader.setControl(m_velocityRequest.withVelocity(velocity));
    }

    public void setShooterVelocity(Double velocity) {
        m_shooterLeader.setControl(m_velocityRequest.withVelocity(velocity));
    }

    //---HOOD (Basic Position Control)
    public Command setHoodPositionCmd(Angle degs) {  
        return runOnce(
            () -> m_hoodSetpoint = degs
        );
    }

    public void setHoodPosition(Angle degrees) {
        m_hoodSetpoint = degrees;
    }


    // The PIDOutput needed to get to the setpoint from the current point
    public void updateHood() {
        // can't read from hardware in Sim, so we read from the hoodSim object
        m_currentHoodPos = m_inSim ? Rotations.of(m_hoodSim.getAngularPositionRotations()) : Rotations.of(m_hoodEncoder.getPosition());

        double hoodPIDOutput = m_hoodPID.calculate(m_currentHoodPos.magnitude(), m_hoodSetpoint.in(Rotations)) * 8;
        
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

    /* SIMULATION STUFF */
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

    public void launchFuel() {
        if (m_fuelStored == 0)
            return;
        // m_fuelStored--;

        AngularVelocity flywheelVelocity = getShooterVelocity();
        Angle hoodAngle = Degrees.of(90).minus(getHoodAngle());
        Angle turretPosition = getTurretPosition();
        LinearVelocity flywheelLinearVelocity = ShotCalculator.angularToLinearVelocity(flywheelVelocity,
                kFlywheelRadius);

        m_fuelSim.launchFuel(
                flywheelLinearVelocity,
                hoodAngle,
                turretPosition,
                kRobotToTurret.getMeasureZ());
    }

    // TODO: update orientation values (if needed)
    private void initSim() {
        WaltMotorSim.initSimFX(m_shooterLeader, ChassisReference.CounterClockwise_Positive,
                TalonFXSimState.MotorType.KrakenX60);
        WaltMotorSim.initSimFX(m_turret, ChassisReference.CounterClockwise_Positive,
                TalonFXSimState.MotorType.KrakenX44);
    }

    /* SOTM METHODS */
    public void setTarget(Translation3d target) {
        currentTarget = target;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            Translation2d flipped = AllianceFlipUtil.apply(target.toTranslation2d());
            currentTarget = new Translation3d(flipped.getX(), flipped.getY(), target.getZ());
        }
    }

    private void setTargetAheadOfRobot(double distanceMeters) {
        Pose2d currentPose = m_poseSupplier.get();

        Translation2d ahead = currentPose.getTranslation().plus(
                new Translation2d(distanceMeters, currentPose.getRotation()));

        // set target at a standard height
        setTarget(new Translation3d(ahead.getX(), ahead.getY(), 2.0));
    }

    private Translation3d getPassingTarget(Pose2d pose) {
        Distance fieldWidth = Inches.of(FieldConstants.fieldWidth);
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        boolean onBlueLeftSide = m_poseSupplier.get().getMeasureY().gt(fieldWidth);

        return isBlue == onBlueLeftSide ? kPassingSpotLeft : kPassingSpotCenter;
    }

    public Command setGoal(TurretGoal goal) {
        return runOnce(() -> {
            m_goal = goal;
            switch (goal) {
                case SCORING:
                    setTarget(FieldConstants.Hub.innerCenterPoint);
                    break;
                case PASSING:
                    setTarget(getPassingTarget(m_poseSupplier.get()));
                    break;
                case TEST:
                    setTargetAheadOfRobot(3);
                    break;
                case OFF:
                    zeroShooterCmd();
            }
        });
    }

    private void calculateShot(Pose2d robot) {
        ChassisSpeeds fieldSpeeds = m_fieldSpeedsSupplier.get();

        ShotData calculatedShot = ShotCalculator.iterativeMovingShotFromInterpolationMap(robot, fieldSpeeds,
                currentTarget, 3);
        Angle azimuthAngle = ShotCalculator.calculateAzimuthAngle(robot, calculatedShot.getTarget(),
                m_turret.getPosition().getValue());
        setTurretPosition(azimuthAngle);
        setHoodPosition(calculatedShot.getHoodAngle());
        setShooterVelocity(
                ShotCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(), kFlywheelRadius));
    }

    /**
     * Version of calculateShot where, FOR TESTING, the turret will align to the
     * target.
     * 
     * @param robot
     */
    private void calculateTurretAngle(Pose2d robot) {
        ChassisSpeeds fieldSpeeds = m_fieldSpeedsSupplier.get();

        ShotData calculatedShot = ShotCalculator.iterativeMovingShotFromInterpolationMap(robot, fieldSpeeds,
                currentTarget, 3);
        Angle azimuthAngle = ShotCalculator.calculateAzimuthAngle(robot, calculatedShot.getTarget(),
                m_turret.getPosition().getValue());
        setTurretPosition(azimuthAngle);
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        // TODO: how on earth are we going to zero the turret? JIG RAHHHHHH
        Pose2d pose = m_poseSupplier.get();

        log_calculateShotCurrPose.accept(pose);

        if (m_goal == TurretGoal.SCORING || m_goal == TurretGoal.PASSING) {
            calculateShot(pose);
        }

        if (m_goal == TurretGoal.TEST) {
            calculateTurretAngle(pose);
        }

        if (m_goal == TurretGoal.PASSING) {
            setTarget(getPassingTarget(pose));
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
            ShotCalculator.angularToLinearVelocity(m_flywheelVelocity, kFlywheelRadius), m_currentHoodPos);
            
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

    public enum ShootingState {
        ACTIVE_SHOOTING,
        TRACKING;
    }

    public enum TurretGoal {
        SCORING,
        PASSING,
        TEST,
        OFF
    }

}