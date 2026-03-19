
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
// import com.reduxrobotics.canand.CanandEventLoop;
// import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.Constants;
import frc.util.WaltMotorSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Shooter extends SubsystemBase {
    /* VARIABLES */
    // boolean m_useShotCalculator = true;

    private AngularVelocity m_flywheelVelocity;
    // private Angle m_turretTurnPosition;

    private int m_fuelStored = 8;

    private final Supplier<Pose2d> m_poseSupplier;
    private final Supplier<ChassisSpeeds> m_fieldSpeedsSupplier;

    private final FuelSim m_fuelSim;

    public final EventLoop homingEventLoop = new EventLoop();

    // ---MOTORS + CONTROL REQUESTS
    private final TalonFX m_shooterA = new TalonFX(kShooterA_CANID, Constants.kShooterBus); // X60Foc
    private final TalonFX m_shooterB = new TalonFX(kShooterB_CANID, Constants.kShooterBus); // X60Foc
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final NeutralOut m_neutralOutReq = new NeutralOut();

    private final Supplier<SwerveDriveState> m_threadsafeSwerveSup;

    private final Hood m_hood = new Hood();
    private final Turret m_turret = new Turret();

    // thread copde
    private Angle m_turretPosition = Rotation.zero();
    private final ShooterCalc m_shooterCalc;
    private final BooleanLogger log_turretHomingHall = new BooleanLogger(kLogTab, "turretHomeHall");

    private Angle m_calcTurret = Rotations.zero();
    private AngularVelocity m_calcFlywheelVelocity = RotationsPerSecond.of(44.81);

    private final DoubleLogger log_calcFlywheelVelocity = new DoubleLogger("Shooter/Flywheel", "calcFlywheelVelocity");
    private final DoubleLogger log_calcTurretPos = new DoubleLogger("Shooter/Turret", "calcTurretPos");

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

    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    private final DoubleLogger log_shooterClosedLoopError = WaltLogger.logDouble("Shooter", "shooterClosedLoopError");

    // private final Tracer m_periodicTracer = new Tracer();

    StatusSignal<Double> sig_shooterCLErr = m_shooterA.getClosedLoopError();

    /* CONSTRUCTOR */
    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<SwerveDriveState> threadsafeSwerveStateSup, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        m_threadsafeSwerveSup = threadsafeSwerveStateSup;
        m_shooterCalc = new ShooterCalc(m_threadsafeSwerveSup, () -> m_turretPosition);

        m_shooterA.getConfigurator().apply(kShooterATalonFXConfiguration);
        m_shooterB.getConfigurator().apply(kShooterBTalonFXConfiguration);
        // m_hood.getConfigurator().apply(kHoodTalonFXSConfiguration);

        m_shooterB.setControl(new Follower(kShooterA_CANID, MotorAlignmentValue.Opposed));

        sig_shooterCLErr.setUpdateFrequency(Hertz.of(50));
        m_flywheelVelocity = m_shooterA.getVelocity().getValue();

        m_poseSupplier = poseSupplier;
        m_fieldSpeedsSupplier = fieldSpeedsSupplier;

        m_fuelSim = FuelSim.getInstance();
        initSim();
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
        return run(() -> setShooterVelocity(m_calcFlywheelVelocity));
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
        sig_shooterCLErr.refresh();
        log_shooterClosedLoopError.accept(sig_shooterCLErr.getValueAsDouble());

        boolean isNear = sig_shooterCLErr.isNear(0, 3);

        log_spunUp.accept(isNear);
        return isNear;
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
        m_turretPosition = m_turret.getCurrTurretPos();
        m_flywheelVelocity = m_shooterA.getVelocity().getValue();

        var calcData = m_shooterCalc.getLatestShotCalcOutputs();

        // set turret reference             // set hood reference  
        if (m_isTurretHomed && m_hood.isHoodHomed()) {
            var turretReference = calcData.turretReference();
            var hoodReference = calcData.hoodReference();

            // set outputs
            var turretVelocityFF = calcData.turretCalcDetails().turretVelocityFF();
            if (m_turret.getTurretLocked()) {
                m_turret.setTurretPos(m_turret.getTurretLockAngle());
                m_hood.setHoodPos(kHoodLockDegs);
                m_calcFlywheelVelocity = kShooterRPS;
            } else {
                if (m_turret.getHoldTurretAtIntake()) {
                    m_turret.setTurretPos(Rotations.of(-0.250));
                } else {
                    m_turret.setTurretPos(turretReference, turretVelocityFF);
                    m_hood.setHoodPos(hoodReference);   //comment out for LERP
                    m_calcFlywheelVelocity = calcData.shooterReference();
                }
            }
        }

        log_shooterVelocityRPS.accept(m_flywheelVelocity.in(RotationsPerSecond));
        log_turretPositionRots.accept(m_turretPosition.in(Rotations));
        log_spunUp.accept(isShooterSpunUp());
        log_calcFlywheelVelocity.accept(m_calcFlywheelVelocity.in(RotationsPerSecond));
        log_calcTurretPos.accept(m_calcTurret.in(Rotations));

        // m_periodicTracer.addEpoch("Logging");

        // m_periodicTracer.printEpochs();
    }

    public void fastPeriodic() {
        homingEventLoop.poll();
    }

    @Override
    public void simulationPeriodic() {
        WaltMotorSim.updateSimFX(m_shooterA, m_shooterSim);
    }

    // public Command turretHomingCmd() {
    //     Runnable init = () -> {
    //         WaltLogger.timedPrint("TurretHoming BEGIN");
    //         m_turretMotor.setControl(m_VoltageReq.withOutput(kHomingVoltage));
    //         m_isTurretHomed = false;
    //         log_turretHomed.accept(m_isTurretHomed);
    //     };

    //     Consumer<Boolean> end = (Boolean interrupted) -> {
    //         if (interrupted) {
    //             m_turretMotor.setControl(m_BrakeReq);
    //             log_turretHomed.accept(m_isTurretHomed);
    //             WaltLogger.timedPrint("TurretHoming INTERRUPTED!!!");
    //             return;
    //         }

    //         m_turretMotor.setPosition(kHomePosition); // Flowkirkentologicalexpialibrostatenuinely
    //         m_turretMotor.setControl(m_BrakeReq);
    //         removeDefaultCommand();
    //         m_isTurretHomed = true;
    //         log_turretHomed.accept(m_isTurretHomed);
    //     };

    //     BooleanSupplier isFinished = () -> {
    //         return !m_turretHomingHall.get() || m_isTurretHomed;
    //     };

    //     return new FunctionalCommand(init, () ->{}, end, isFinished, this);
    // }

    public Command homingCmds() {
        return Commands.sequence(
            // hoodCurrentSenseHomingCmd().asProxy(),
            // turretHomingCmd().asProxy()
        );
    }
}