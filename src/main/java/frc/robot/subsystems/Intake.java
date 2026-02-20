package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.ChassisReference;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.WaltLogger.DoubleLogger;

import frc.robot.Constants.IntakeK;
import frc.util.WaltMotorSim;
import frc.util.WaltLogger;

public class Intake extends SubsystemBase {
    /* CLASS VARIABLES */
    //---MOTORS + CONTROL REQUESTS
    private final TalonFX m_intakeArm = new TalonFX(IntakeK.kIntakeArmCANID);   //X44Foc
    private final TalonFX m_intakeRollers = new TalonFX(IntakeK.kIntakeRollersCANID);   //X44Foc

    private MotionMagicVoltage m_MMVReq = new MotionMagicVoltage(0).withEnableFOC(true);
    private VelocityVoltage m_VVReq = new VelocityVoltage(0).withEnableFOC(true);

    /* SIM OBJECTS */
    private final DCMotorSim m_intakeArmSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1),
            IntakeK.kIntakeArmMOI,
            IntakeK.kIntakeArmGearing
        ),
        DCMotor.getKrakenX44Foc(1)
    );

    private final DCMotorSim m_intakeRollersSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1),
            IntakeK.kIntakeRollersMOI,
            IntakeK.kIntakeRollersGearing
        ),
        DCMotor.getKrakenX44Foc(1) // returns gearbox
    );

    /* LOGGERS */
    DoubleLogger log_intakeArmRots = WaltLogger.logDouble(IntakeK.kLogTab, "intakeArmRots");
    DoubleLogger log_targetIntakeArmRots = WaltLogger.logDouble(IntakeK.kLogTab, "targetIntakeArmRots");

    DoubleLogger log_intakeRollersRPS = WaltLogger.logDouble(IntakeK.kLogTab, "intakeRollersRPS");
    DoubleLogger log_targetIntakeRollersRPS = WaltLogger.logDouble(IntakeK.kLogTab, "targetIntakeRollersRPS");

    /* CONSTRUCTOR */
    public Intake() {
        m_intakeArm.getConfigurator().apply(IntakeK.kIntakeArmConfiguration);
        m_intakeRollers.getConfigurator().apply(IntakeK.kIntakeRollersConfiguration);
        initSim();
    }

    private void initSim() {
        WaltMotorSim.initSimFX(m_intakeArm, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX44);
        WaltMotorSim.initSimFX(m_intakeRollers, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX44);
    }

    /* COMMANDS */
    public Command setIntakeArmPos(IntakeArmPosition rots) {
        return setIntakeArmPos(rots.rots);
    }

    public Command setIntakeArmPos(Angle rots) {
        return runOnce(() -> m_intakeArm.setControl(m_MMVReq.withPosition(rots)));
    }

    //for TestingDashboard
    public Command setIntakeArmPos(DoubleSubscriber sub_rots) {
        return run(() -> m_intakeArm.setControl(m_MMVReq.withPosition(Rotations.of(sub_rots.get()))));
    }

    public Command setIntakeRollersSpeed(IntakeRollersVelocity RPS) {
        return setIntakeRollersSpeed(RPS.RPS);
    }

    public Command setIntakeRollersSpeed(AngularVelocity RPS) {
        return runOnce(() -> m_intakeRollers.setControl(m_VVReq.withVelocity(RPS)));
    }

    //for TestingDashboard
    public Command setIntakeRollersSpeed(DoubleSubscriber sub_RPS) {
        return run(() -> m_intakeRollers.setControl(m_VVReq.withVelocity(RotationsPerSecond.of(sub_RPS.get()))));
    }

    public TalonFX getIntakeArmMotor() {
        return m_intakeArm;
    }

    public TalonFX getIntakeRollers() {
        return m_intakeRollers;
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        log_targetIntakeArmRots.accept(m_MMVReq.Position);
        log_targetIntakeRollersRPS.accept(m_VVReq.Velocity);
        log_intakeRollersRPS.accept(m_intakeRollers.getVelocity().getValueAsDouble());
        log_intakeArmRots.accept(m_intakeArm.getPosition().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        WaltMotorSim.updateSimFX(m_intakeArm, m_intakeArmSim);
        WaltMotorSim.updateSimFX(m_intakeRollers, m_intakeRollersSim);
    }

    /* ENUMS */
    public enum IntakeArmPosition{
        RETRACTED(0),
        SAFE(72),
        DEPLOYED(87.485);

        public Angle degs;
        public Angle rots;

        private IntakeArmPosition(double degs) {
            this.degs = Degrees.of(degs);
            this.rots = Rotations.of(this.degs.in(Rotations));
        }
    }

    public enum IntakeRollersVelocity{
        MAX(50),
        MID(33),
        STOP(0);

        private AngularVelocity RPS;

        private IntakeRollersVelocity(double RPS) {
            this.RPS = RotationsPerSecond.of(RPS);
        }
    }
}