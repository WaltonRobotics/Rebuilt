
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotation;
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
    Angle m_turretLockAngle = Degrees.zero();

    private AngularVelocity m_flywheelVelocity;
    // private Angle m_turretTurnPosition;

    private int m_fuelStored = 8;

    private final Supplier<Pose2d> m_poseSupplier;
    private final Supplier<ChassisSpeeds> m_fieldSpeedsSupplier;

    // private final TurretVisualizer m_turretVisualizer;
    // private final FuelSim m_fuelSim;

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
    private Angle m_turretPosition = Rotation.zero();
    // private StatusSignal<Angle> sig_turretPos_THREADONLY = m_turret.getPosition(false).clone();
    private final ShooterCalc m_shooterCalc;

    private final DigitalInput m_turretHomingHall = new DigitalInput(2);
    private final Trigger trg_homingHallDirect = new Trigger(homingEventLoop, () -> !m_turretHomingHall.get());
    private final BooleanLogger log_turretHomingHall = new BooleanLogger(kLogTab, "turretHomeHall");

    private Angle m_calcTurret = Rotations.zero();
    private double m_calcdFlywheelVelocityRps = 44.81;
    private double m_driverRPSTweak = 0.0;

    // Precomputed: true if turret travel is less than one full rotation (sub-360
    // cope path)
    private static final boolean kTurretSubRotation = kTurretMaxRotsFromHome.times(2).magnitude() < 1.0;
    // Precomputed doubles for hot-path unit conversions
    private static final double kTurretMinRotsD = kTurretMinRots.in(Rotations);
    private static final double kTurretMinRotsMagnitudeD = kTurretMinRots.magnitude();
    private static final double kTurretMaxRotsD = kTurretMaxRots.in(Rotations);
    private static final double kTurretMaxRotsMagnitudeD = kTurretMaxRots.magnitude();

    private final DoubleLogger log_calcFlywheelVelocity = new DoubleLogger("Shooter/Flywheel", "calcFlywheelVelocity");
    private final DoubleLogger log_calcTurretPos = new DoubleLogger("Shooter/Turret", "calcTurretPos");
    private final DoubleLogger log_driverAddedRPS = WaltLogger.logDouble(kLogTab, "driverAddedRPS");


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
    // private final DoubleLogger log_hoodControlPos = WaltLogger.logDouble("Shooter/Hood", "hoodControlPos");

    // private final Tracer m_periodicTracer = new Tracer();

    StatusSignal<Double> sig_shooterCLErr = m_shooterA.getClosedLoopError();

    private final Command m_homingCommand = turretHomingCmd();

    /* CONSTRUCTOR */
    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<SwerveDriveState> threadsafeSwerveStateSup, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        m_threadsafeSwerveSup = threadsafeSwerveStateSup;
        m_shooterCalc = new ShooterCalc(m_threadsafeSwerveSup, () -> m_turretPosition);

        m_shooterA.getConfigurator().apply(kShooterATalonFXConfiguration);
        m_shooterB.getConfigurator().apply(kShooterBTalonFXConfiguration);
        m_turretMotor.getConfigurator().apply(kTurretTalonFXConfiguration);
        // m_hood.getConfigurator().apply(kHoodTalonFXSConfiguration);

        m_shooterB.setControl(new Follower(kShooterA_CANID, MotorAlignmentValue.Opposed));

        sig_shooterCLErr.setUpdateFrequency(Hertz.of(50));

        Angle turretTurnPosition = m_turretMotor.getPosition().getValue();
        m_flywheelVelocity = m_shooterA.getVelocity().getValue();

        m_poseSupplier = poseSupplier;
        m_fieldSpeedsSupplier = fieldSpeedsSupplier;
        setDefaultCommand(m_homingCommand);

        // m_turretVisualizer = new TurretVisualizer(
        //         () -> new Pose3d(m_poseSupplier.get().rotateAround(
        //                 poseSupplier.get().getTranslation(), new Rotation2d(turretTurnPosition)))
        //                 .transformBy(kTurretTransform),
        //         fieldSpeedsSupplier);

        // m_fuelSim = FuelSim.getInstance();

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
        System.out.println("setIntaking: " + intaking);
        m_holdTurretAtIntakePos = intaking;
    }

    public void setTurretLock(boolean locked) {
        m_turretLocked = locked;
        if (m_turretLocked) {
            m_turretLockAngle = m_turretMotor.getPosition().getValue();
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
        return run(() -> setShooterVelocity(m_calcdFlywheelVelocityRps));
    }

    public Command driverRPSAlter(boolean increase) {
        return Commands.runOnce(() -> {
            m_driverRPSTweak += kDriverRPSIncreaseD * (increase ? 1 : -1);;
            log_driverAddedRPS.accept(m_driverRPSTweak);
        });
    }

    public void setShooterVelocity(AngularVelocity RPS) {
        setShooterVelocity(RPS.in(RotationsPerSecond));
    }

    public void setShooterVelocity(double rotPerSec) {
        if (rotPerSec == 0) {
            m_shooterA.setControl(m_neutralOutReq);
        } else {
            m_shooterA.setControl(m_velocityRequest.withVelocity(rotPerSec));
        }
    }


    // for TestingDashboard
    public Command setShooterVelocityCmd(DoubleSubscriber sub_RPS) {
        return run(() -> setShooterVelocity(RotationsPerSecond.of(sub_RPS.get())));
    }

    public boolean isShooterSpunUp() {
        sig_shooterCLErr.refresh();
        log_shooterClosedLoopError.accept(sig_shooterCLErr.getValueAsDouble());

        boolean isNear = sig_shooterCLErr.isNear(0, 3);

        log_spunUp.accept(isNear);
        return isNear;
    }

    // ---TURRET (Motionmagic Angle Control)
    public Command setTurretPosCmd(Angle rots) {
        return runOnce(() -> setTurretPos(rots));
    }

    public void setTurretPos(Angle rots) {
        setTurretPos(rots, RotationsPerSecond.zero());
    }

    public void setTurretPos(Angle rots, AngularVelocity velocityFF) {
        if (!m_isTurretHomed) { return; }
        m_turretMotor.setControl(m_PVRequest.withPosition(rots).withVelocity(velocityFF));
        log_turretControlPos.accept(rots.in(Rotations));
    }

    public void setTurretNeutralMode(NeutralModeValue value) {
        m_turretMotor.setNeutralMode(value);
    }

    // for TestingDashboard
    public Command setTurretPositionCmd(DoubleSubscriber sub_rots) {
        return run(() -> setTurretPos(Rotations.of(sub_rots.get())));
    }

    /* GETTERS */
    public AngularVelocity getShooterVelocity() {
        return m_flywheelVelocity;
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
        m_turretPosition = m_turretMotor.getPosition().getValue();
        m_flywheelVelocity = m_shooterA.getVelocity().getValue();

        var calcData = m_shooterCalc.getLatestShotCalcOutputs();

        // set turret reference             // set hood reference  
        if (m_isTurretHomed && m_hood.isHoodHomed()) {
            var turretReference = calcData.turretReference();
            var hoodReference = calcData.hoodReference();

            // set outputs
            var turretVelocityFF = calcData.turretCalcDetails().turretVelocityFF();
            if (m_turretLocked) {
                setTurretPos(m_turretLockAngle);
                m_hood.setHoodPos(kHoodLockDegs);
                m_calcdFlywheelVelocityRps = kShooterRPSd;
            } else {
                if (m_holdTurretAtIntakePos) {
                    setTurretPos(Rotations.of(-0.250));
                } else {
                    setTurretPos(turretReference, turretVelocityFF);
                    m_hood.setHoodPos(hoodReference);
                    m_calcdFlywheelVelocityRps = calcData.shooterReferenceRps();
                    if (true) { // ENABLE THIS TO ALLOW DRIVER RPS TWEAK
                        m_calcdFlywheelVelocityRps += m_driverRPSTweak;
                    }
                }
            }
        }

        log_shooterVelocityRPS.accept(m_flywheelVelocity.in(RotationsPerSecond));
        log_turretPositionRots.accept(m_turretPosition.in(Rotations));
        log_spunUp.accept(isShooterSpunUp());
        log_calcFlywheelVelocity.accept(m_calcdFlywheelVelocityRps);
        log_calcTurretPos.accept(m_calcTurret.in(Rotations));

        log_turretHomingHall.accept(trg_homingHallDirect.getAsBoolean());

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