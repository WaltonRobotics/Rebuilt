
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
// import com.reduxrobotics.canand.CanandEventLoop;
// import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import frc.robot.Constants;
import frc.util.WaltMotorSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Shooter extends SubsystemBase {
    /* VARIABLES */
    // boolean m_useShotCalculator = true;
    boolean m_holdTurretAtIntakePos = false;
    boolean m_turretLocked = false;
    double m_turretLockAngleRots = 0.0;

    private double m_latestFlywheelVelocityRotPerSec;
    private boolean m_isShooterSpunUp = false;

    private int m_fuelStored = 8;

    private final Supplier<Pose2d> m_poseSupplier;
    private final Supplier<ChassisSpeeds> m_fieldSpeedsSupplier;

    private final TurretVisualizer m_turretVisualizer;
    private final FuelSim m_fuelSim;

    public final EventLoop homingEventLoop = new EventLoop();

    // ---MOTORS + CONTROL REQUESTS
    private final TalonFX m_shooterA = new TalonFX(kShooterA_CANID, Constants.kShooterBus); // X60Foc
    private final TalonFX m_shooterB = new TalonFX(kShooterB_CANID, Constants.kShooterBus); // X60Foc
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final NeutralOut m_neutralOutReq = new NeutralOut();

    private final TalonFX m_turretMotor = new TalonFX(kTurretCANID, Constants.kCanivoreBus); // X44Foc
    private final PositionVoltage m_PVRequest = new PositionVoltage(0).withEnableFOC(true);
    private final VoltageOut m_VoltageReq = new VoltageOut(0);
    private final StaticBrake m_BrakeReq = new StaticBrake();

    // private final TalonFXS m_hood = new TalonFXS(kHoodCANID);
    // private final PositionVoltage m_hoodPVRequest = new PositionVoltage(0).withEnableFOC(false);
    // private final VoltageOut m_hoodZeroReq = new VoltageOut(0);

    private final Supplier<SwerveDriveState> m_threadsafeSwerveSup;

    private final Hood m_hood = new Hood();
    private final Turret m_turret = new Turret();

    // thread copde
    private double m_latestTurretPositionRots = 0.0;
    // private StatusSignal<Angle> sig_turretPos_THREADONLY = m_turret.getPosition(false).clone();
    private final ShooterCalc m_shooterCalc;


    private final DigitalInput m_turretHomingHall = new DigitalInput(2);
    private final Trigger trg_homingHallDirect = new Trigger(homingEventLoop, () -> !m_turretHomingHall.get());
    private final BooleanLogger log_turretHomingHall = new BooleanLogger(kLogTab, "turretHomeHall");

    private double m_calcTurretRots = 0.0;
    private double m_calcFlywheelVelocityRotPerSec = 44.81;

    private static final double kIntakeTurretPosRots = -0.250;

    private final DoubleLogger log_calcFlywheelVelocity = new DoubleLogger("Shooter/Flywheel", "calcFlywheelVelocity");
    private final DoubleLogger log_calcTurretPos = new DoubleLogger("Shooter/Turret", "calcTurretPos");


    // private static final DoubleLogger log_timeOfFlight = new
    // DoubleLogger("Shooter/Calculator", "timeOfFlight");


    private final BooleanLogger log_turretHomed = WaltLogger.logBoolean("Shooter/Turret", "Homed");
    // ---LOGIC BOOLEANS
    private boolean m_isTurretHomed = false;
    public BooleanSupplier turretHomedSupp = () -> m_isTurretHomed;
    // private boolean m_isHoodHomed = false;

    /* SIM OBJECTS */
    private final FlywheelSim m_shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(2), kShooterMoI, kShooterGearing), DCMotor.getKrakenX60Foc(2) // returns gearbox
    );

    private final DCMotorSim m_turretSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1), kTurretMoI, kTurretGearing), DCMotor.getKrakenX44Foc(1) // returns gearbox
    );

    /* LOGGERS */
    private final DoubleLogger log_shooterVelocityRPS = WaltLogger.logDouble("Shooter/Flywheel", "shooterVelocityRPS");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble("Shooter/Turret", "turretPositionRots");

    // private final BooleanLogger log_exitBeamBreak =
    // WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    private final DoubleLogger log_shooterClosedLoopError = WaltLogger.logDouble("Shooter", "shooterClosedLoopError");

    private final DoubleLogger log_turretControlPos = WaltLogger.logDouble("Shooter/Turret", "turretControlPos");
    private final DoubleLogger log_turretControlVeloFF = WaltLogger.logDouble("Shooter/Turret", "turretCtrlVeloFF");
    // private final DoubleLogger log_hoodControlPos = WaltLogger.logDouble("Shooter/Hood", "hoodControlPos");

    // private final Tracer m_periodicTracer = new Tracer();

    StatusSignal<Double> sig_shooterCLErr = m_shooterA.getClosedLoopError();

    private final Command m_homingCommand = turretHomingCmd();

    /* CONSTRUCTOR */
    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<SwerveDriveState> threadsafeSwerveStateSup, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        m_threadsafeSwerveSup = threadsafeSwerveStateSup;
        m_shooterCalc = new ShooterCalc(m_threadsafeSwerveSup, () -> m_latestTurretPositionRots);

        m_shooterA.getConfigurator().apply(kShooterATalonFXConfiguration);
        m_shooterB.getConfigurator().apply(kShooterBTalonFXConfiguration);
        m_turretMotor.getConfigurator().apply(kTurretTalonFXConfiguration);
        // m_hood.getConfigurator().apply(kHoodTalonFXSConfiguration);

        m_shooterB.setControl(new Follower(kShooterA_CANID, MotorAlignmentValue.Opposed));

        sig_shooterCLErr.setUpdateFrequency(Hertz.of(50));

        double turretTurnPosition = m_turretMotor.getPosition().getValueAsDouble();
        m_latestFlywheelVelocityRotPerSec = m_shooterA.getVelocity().getValueAsDouble();

        m_poseSupplier = poseSupplier;
        m_fieldSpeedsSupplier = fieldSpeedsSupplier;
        setDefaultCommand(m_homingCommand);

        m_turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(m_poseSupplier.get().rotateAround(
                        poseSupplier.get().getTranslation(), new Rotation2d(turretTurnPosition * 2.0 * Math.PI)))
                        .transformBy(kTurretTransform),
                fieldSpeedsSupplier);

        m_fuelSim = FuelSim.getInstance();

        trg_homingHallDirect.onTrue(Commands.sequence(
                Commands.runOnce(() -> {
                    m_homingCommand.cancel();
                    WaltLogger.timedPrint("FastHoming OK!!!");
                }),
                Commands.runOnce(() -> homingEventLoop.clear())));

        m_turretMotor.setPosition(kInitPosition);

        initSim();
    }

    // public Command setShotCalcCmd(boolean enable) {
    //     return Commands.runOnce(() -> {
    //         m_useShotCalculator = enable;
    //     });
    // }

    // public void setShotCalc(boolean enable) {
    //     m_useShotCalculator = enable;
    // }

    public void setIntaking(boolean intaking) {
        m_holdTurretAtIntakePos = intaking;
    }

    public void setTurretLock(boolean locked) {
        m_turretLocked = locked;
        if (m_turretLocked) {
            m_turretLockAngleRots = m_latestTurretPositionRots;
        }
    }

    public Command setTurretLockCmd(boolean locked) {
        return runOnce(() -> setTurretLock(locked));
    }

    // ---SHOOTER (Velocity Control)
    public Command setShooterVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> setShooterVelocity(RPS));
    }

    public Command setShooterVelocityCmdSupp(Supplier<AngularVelocity> supp_RPS) {
        return runOnce(() -> setShooterVelocity(supp_RPS.get()));
    }

    public void setShooterVelocitySupp(Supplier<AngularVelocity> supp_RPS) {
        run(() -> setShooterVelocity(supp_RPS.get()));
    }

    public Command shootFromCalc() {
        return run(() -> m_shooterA.setControl(m_velocityRequest.withVelocity(m_calcFlywheelVelocityRotPerSec)));
    }

    public void setShooterVelocity(AngularVelocity RPS) {
        if (RPS.isEquivalent(RotationsPerSecond.zero())) {
            m_shooterA.setControl(m_neutralOutReq);
        } else {
            m_shooterA.setControl(m_velocityRequest.withVelocity(RPS));
        }
    }


    // for TestingDashboard
    public Command setShooterVelocityCmd(DoubleSubscriber sub_RPS) {
        return run(() -> setShooterVelocity(RotationsPerSecond.of(sub_RPS.get())));
    }

    public boolean isShooterSpunUp() {
        return m_isShooterSpunUp;
    }

    public void setTurretPos(double positionRots, double velocityFFRotPerSec) {
        if (!m_isTurretHomed) { return; }
        m_turretMotor.setControl(m_PVRequest.withPosition(positionRots).withVelocity(velocityFFRotPerSec));
        log_turretControlPos.accept(positionRots);
        log_turretControlVeloFF.accept(velocityFFRotPerSec);
    }

    public void setTurretNeutralMode(NeutralModeValue value) {
        m_turretMotor.setNeutralMode(value);
    }

    /* GETTERS */
    public double getShooterVelocityRotPerSec() {
        return m_latestFlywheelVelocityRotPerSec;
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

    // /**
    //  * Launches SIMULATION FUEL™ at the current Flywheel Velocity, current Hood
    //  * Angle, and the
    //  * current Turret Position.
    //  */
    // public void launchFuel() {
    //     if (m_fuelStored == 0)
    //         return;
    //     // m_fuelStored--;

    //     AngularVelocity flywheelVelocity = getShooterVelocity(); // current flywheel velocity
    //     Angle hoodAngle = Degrees.of(90).minus(getHoodAngle()); // current hood angle (need to subtract from 90 to get
    //                                                             // an accurate launching angle)
    //     Angle turretPosition = getTurretPosition(); // current turret position
    //     LinearVelocity flywheelLinearVelocity = ShotCalculator
    //             .angularToLinearVelocity(flywheelVelocity, kFlywheelRadius);

    //     m_fuelSim.launchFuel(flywheelLinearVelocity, hoodAngle, turretPosition,
    //             kTurretTransform.getMeasureZ()); // launch from where the turret *should* be
    // }

    // TODO: update orientation values (if needed)
    private void initSim() {
        WaltMotorSim.initSimFX(m_shooterA, ChassisReference.CounterClockwise_Positive,
                TalonFXSimState.MotorType.KrakenX60);
        WaltMotorSim.initSimFX(m_turretMotor, ChassisReference.CounterClockwise_Positive,
                TalonFXSimState.MotorType.KrakenX44);
    }

    /* ShootOnTheMove™ */

    /**
     * Tells us whether or not our Turret, Hood, and Flywheel are at their desired
     * position with a certain amount of tolerance relative to the object
     * 
     * Hood Tolerance: .5 degrees. Turret Tolerance: __ rotations. Flywheel
     * Tolerance: __ RPS.
     * 
     * @return
     */
    // public boolean atPosition() {
    // var turretAtPos = getTurretPosition().isNear(m_calcTurret, Degrees.of(1));
    // //TODO: get the turret tolerance
    // var flywheelAtPos = getShooterVelocity().isNear(m_calcFlywheelVelocity,
    // RotationsPerSecond.of(1)); //TODO: get the flywheel tolerance
    // var hoodAtPos =
    // m_hoodEncoder.getVelocity().getValue().isNear(RotationsPerSecond.of(0),
    // 0.01); //is this right buh
    // return turretAtPos && flywheelAtPos && hoodAtPos;
    // }

    /* PERIODICS */
    @Override
    public void periodic() {
        m_turret.periodic();
        // m_periodicTracer.addEpoch("Entry (Unused Time)");

        // Cache all signals at the top so every consumer in this loop sees the same
        // values
        // THIS IS USED SNEAKILY BY SHOTCALC DO NOT MOVE THIS
        m_latestTurretPositionRots = m_turretMotor.getPosition().getValueAsDouble();
        m_latestFlywheelVelocityRotPerSec = m_shooterA.getVelocity().getValueAsDouble();
        sig_shooterCLErr.refresh();
        log_shooterClosedLoopError.accept(sig_shooterCLErr.getValueAsDouble());
        m_isShooterSpunUp = sig_shooterCLErr.isNear(0, 3);

        var calcData = m_shooterCalc.getLatestShotCalcOutputs();

        // All calcData fields are CTRE-native (rotations, rot/s) — no wrapping needed
        if (m_isTurretHomed && m_hood.isHoodHomed()) {
            if (m_turretLocked) {
                setTurretPos(m_turretLockAngleRots, 0.0);
                m_hood.setHoodPos(kHoodLockDegs);
                m_calcFlywheelVelocityRotPerSec = kShooterRPSd;
            } else {
                if (m_holdTurretAtIntakePos) {
                    setTurretPos(kIntakeTurretPosRots, 0.0);
                } else {
                    setTurretPos(calcData.turretReferenceRots(), calcData.turretCalcDetails().turretVelocityFFRotPerSec());
                    m_hood.setHoodPos(calcData.hoodReferenceRots());
                    m_calcFlywheelVelocityRotPerSec = calcData.shooterReferenceRotPerSec();
                }
            }
        }

        log_shooterVelocityRPS.accept(m_latestFlywheelVelocityRotPerSec);
        log_turretPositionRots.accept(m_latestTurretPositionRots);
        log_spunUp.accept(m_isShooterSpunUp);
        log_calcFlywheelVelocity.accept(m_calcFlywheelVelocityRotPerSec);
        log_calcTurretPos.accept(m_calcTurretRots);

        // m_periodicTracer.addEpoch("Logging");

        // m_periodicTracer.printEpochs();
    }

    public void fastPeriodic() {
        homingEventLoop.poll();
    }

    @Override
    public void simulationPeriodic() {
        // m_turretVisualizer.updateFuel(
        //         ShotCalculator.angularToLinearVelocity(m_flywheelVelocity, kFlywheelRadius),
        //         getHoodAngle());

        WaltMotorSim.updateSimFX(m_shooterA, m_shooterSim);
        WaltMotorSim.updateSimFX(m_turretMotor, m_turretSim);
        // WaltMotorSim.updateSimServo(m_hood, m_hoodSim);
    }

    public Command turretHomingCmd() {
        Runnable init = () -> {
            WaltLogger.timedPrint("TurretHoming BEGIN");
            m_turretMotor.setControl(m_VoltageReq.withOutput(kHomingVoltage));
            m_isTurretHomed = false;
            log_turretHomed.accept(m_isTurretHomed);
        };

        Consumer<Boolean> end = (Boolean interrupted) -> {
            if (interrupted) {
                m_turretMotor.setControl(m_BrakeReq);
                log_turretHomed.accept(m_isTurretHomed);
                WaltLogger.timedPrint("TurretHoming INTERRUPTED!!!");
                return;
            }

            m_turretMotor.setPosition(kHomePosition); // Flowkirkentologicalexpialibrostatenuinely
            m_turretMotor.setControl(m_BrakeReq);
            removeDefaultCommand();
            m_isTurretHomed = true;
            log_turretHomed.accept(m_isTurretHomed);
        };

        BooleanSupplier isFinished = () -> {
            return !m_turretHomingHall.get() || m_isTurretHomed;
        };

        return new FunctionalCommand(init, () ->{}, end, isFinished, this);
    }

    public Command homingCmds() {
        return Commands.sequence(
            // hoodCurrentSenseHomingCmd().asProxy(),
            turretHomingCmd().asProxy()
        );
    }
}