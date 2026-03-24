
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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterCalc.ShotCalcOutputs;
import frc.util.WaltMotorSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Shooter extends SubsystemBase {
    /* VARIABLES */
    // boolean m_useShotCalculator = true;

    private double m_latestFlywheelVelocityRotPerSec;
    private boolean m_isShooterSpunUp = false;

    private int m_fuelStored = 8;

    private final Supplier<Pose2d> m_poseSupplier;
    private final Supplier<ChassisSpeeds> m_fieldSpeedsSupplier;

    // private final TurretVisualizer m_turretVisualizer;
    // private final FuelSim m_fuelSim;

    // ---MOTORS + CONTROL REQUESTS
    private final TalonFX m_shooterA = new TalonFX(kShooterA_CANID, Constants.kShooterBus); // X60Foc
    private final TalonFX m_shooterB = new TalonFX(kShooterB_CANID, Constants.kShooterBus); // X60Foc
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final NeutralOut m_neutralOutReq = new NeutralOut();

    private final Supplier<SwerveDriveState> m_threadsafeSwerveSup;

    public final Hood m_hood;
    public final Turret m_turret;

    // thread copde
    private double m_latestTurretPositionRots = 0.0;
    private final ShooterCalc m_shooterCalc;
    private final BooleanLogger log_turretHomingHall = new BooleanLogger(kLogTab, "turretHomeHall");

    private double m_calcTurretRots = 0.0;
    private double m_calcFlywheelVelocityRotPerSec = 44.81;

    private final DoubleLogger log_calcFlywheelVelocity = new DoubleLogger("Shooter/Flywheel", "calcFlywheelVelocity");
    private final DoubleLogger log_calcTurretPos = new DoubleLogger("Shooter/Turret", "calcTurretPos");
    private final DoubleLogger log_driverAddedRPS = WaltLogger.logDouble(kLogTab, "driverAddedRPS");

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
        m_hood = new Hood();
        m_turret = new Turret();
        m_threadsafeSwerveSup = threadsafeSwerveStateSup;
        m_shooterCalc = new ShooterCalc(m_threadsafeSwerveSup, () -> m_latestTurretPositionRots);

        m_shooterA.getConfigurator().apply(kShooterATalonFXConfiguration);
        m_shooterB.getConfigurator().apply(kShooterBTalonFXConfiguration);
        // m_hood.getConfigurator().apply(kHoodTalonFXSConfiguration);

        m_shooterB.setControl(new Follower(kShooterA_CANID, MotorAlignmentValue.Opposed));

        sig_shooterCLErr.setUpdateFrequency(Hertz.of(50));
        m_latestFlywheelVelocityRotPerSec = m_shooterA.getVelocity().getValueAsDouble();

        m_poseSupplier = poseSupplier;
        m_fieldSpeedsSupplier = fieldSpeedsSupplier;

        // MANUAL HOMING REQUIRED
        // setDefaultCommand(m_homingCommand);
        // m_turretMotor.setPosition(kInitPosition);
        m_isTurretHomed = true;

        // m_turretVisualizer = new TurretVisualizer(
        //         () -> new Pose3d(m_poseSupplier.get().rotateAround(
        //                 poseSupplier.get().getTranslation(), new Rotation2d(turretTurnPosition)))
        //                 .transformBy(kTurretTransform),
        //         fieldSpeedsSupplier);

        // m_fuelSim = FuelSim.getInstance();

        // trg_homingHallDirect.onTrue(Commands.sequence(
        //         Commands.runOnce(() -> {
        //             m_homingCommand.cancel();
        //             WaltLogger.timedPrint("FastHoming OK!!!");
        //         }),
        //         Commands.runOnce(() -> homingEventLoop.clear())));


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
        // return run(() -> setShooterVelocity(m_calcdFlywheelVelocityRps));
    }

    // public Command driverRPSIncreaseWhileHeldCmd() {
    //     return Commands.runOnce(() -> {
    //         driverRPSAlterStatic(true);
    //     }).finallyDo(() -> driverResetRPSAlter());
    // }

    // public Command driverRPSAlterDynamic(boolean increase) {
    //     return Commands.runOnce(() -> {
    //         m_driverRPSTweak = increase ? (m_calcdFlywheelVelocityRps * 0.05) : (m_calcdFlywheelVelocityRps * -0.05);
    //         log_driverAddedRPS.accept(m_driverRPSTweak);
    //     });
    // }

    // public Command driverRPSAlterStatic(boolean increase) {
    //     return Commands.runOnce(() -> {
    //         m_driverRPSTweak += kDriverRPSIncreaseD * (increase ? 1 : -1);
    //         log_driverAddedRPS.accept(m_driverRPSTweak);
    //     });
    // }

    // public Command driverResetRPSAlter() {
    //     return Commands.runOnce(() -> m_driverRPSTweak = 0);
    // }

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
        return m_isShooterSpunUp;
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

        // Cache all signals at the top so every consumer in this loop sees the same values
        // THIS IS USED SNEAKILY BY SHOTCALC DO NOT MOVE THIS
        m_latestTurretPositionRots = m_turret.getCurrTurretPos().in(Rotations);
        m_latestFlywheelVelocityRotPerSec = m_shooterA.getVelocity().getValue().in(RotationsPerSecond);

        ShotCalcOutputs calcData = m_shooterCalc.getLatestShotCalcOutputs();

        // set turret reference
        if (m_turret.isTurretHomed()) {
            var turretReference = calcData.turretReferenceRots();

            // set outputs
            var turretVelocityFF = calcData.turretCalcDetails().turretVelocityFF();
            if (m_turret.getTurretLocked()) {
                m_turret.setTurretPos(m_turret.getTurretLockAngle());
                // m_calcdFlywheelVelocityRps = kShooterRPSd;
            } else {
                if (m_turret.getHoldTurretAtIntake()) {
                    m_turret.setTurretPos(Rotations.of(-0.250));
                } else {
                    m_turret.setTurretPos(turretReference, turretVelocityFF);
                    // m_calcdFlywheelVelocityRps = calcData.shooterReferenceRps();
                    if (true) { // ENABLE THIS TO ALLOW DRIVER RPS TWEAK
                        // m_calcdFlywheelVelocityRps += m_driverRPSTweak;
                        // m_calcdFlywheelVelocityRps = MathUtil.clamp(m_calcdFlywheelVelocityRps, 0, kShooterMaxRPSd);    //clamp here or clamp only when setShooterVel is called?
                    }
                }
            }
        }

        if (m_hood.isHoodHomed()) {
            double hoodReference = calcData.hoodReference();

            if (m_turret.getTurretLocked()) {
                m_hood.setHoodPos(kHoodLockRots_double);
            } else {
                if (!m_turret.getHoldTurretAtIntake()) {
                    m_hood.setHoodPos(hoodReference);
                }
            }
        }

        // log_shooterVelocityRPS.accept(m_flywheelVelocity.in(RotationsPerSecond));
        log_turretPositionRots.accept(m_latestTurretPositionRots);
        // log_spunUp.accept(isShooterSpunUp());
        // log_calcFlywheelVelocity.accept(m_calcdFlywheelVelocityRps);
        log_calcTurretPos.accept(m_calcTurretRots);

        // log_turretHomingHall.accept(trg_homingHallDirect.getAsBoolean());

        // m_periodicTracer.addEpoch("Logging");

        // m_periodicTracer.printEpochs();
    }

    @Override
    public void simulationPeriodic() {
        WaltMotorSim.updateSimFX(m_shooterA, m_shooterSim);
    }

    // public Command homingCmds() {
    //     return Commands.sequence(
    //         // hoodCurrentSenseHomingCmd().asProxy(),
    //         turretHomingCmd().asProxy()
    //     );
    // } 
}