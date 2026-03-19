package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterK.*;
import static frc.robot.Constants.TurretK.kLogTab;
import static frc.robot.Constants.TurretK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.IntLogger;

public class Turret extends SubsystemBase {
    private boolean m_holdTurretAtIntakePos = false;
    private boolean m_turretLocked = false;
    private Angle m_turretLockAngle = Degrees.zero();

    public final EventLoop homingEventLoop = new EventLoop();

    private final TalonFX m_turretMotor = new TalonFX(kTurretCANID, Constants.kCanivoreBus); // X44Foc
    private final PositionVoltage m_PVRequest = new PositionVoltage(0).withEnableFOC(true);
    private final VoltageOut m_VoltageReq = new VoltageOut(0);
    private final StaticBrake m_BrakeReq = new StaticBrake();

    // Precomputed: true if turret travel is less than one full rotation (sub-360
    // cope path)
    private static final boolean kTurretSubRotation = kTurretMaxRotsFromHome.times(2).magnitude() < 1.0;
    // Precomputed doubles for hot-path unit conversions
    private static final double kTurretMinRotsD = kTurretMinRots.in(Rotations);
    private static final double kTurretMinRotsMagnitudeD = kTurretMinRots.magnitude();
    private static final double kTurretMaxRotsD = kTurretMaxRots.in(Rotations);
    private static final double kTurretMaxRotsMagnitudeD = kTurretMaxRots.magnitude();

    private final Command m_homingCommand = turretHomingCmd();

    private final BooleanLogger log_turretHomed = WaltLogger.logBoolean("Shooter/Turret", "Homed");
    // ---LOGIC BOOLEANS
    private boolean m_isTurretHomed = false;
    public BooleanSupplier turretHomedSupp = () -> m_isTurretHomed;

    private final CANcoder m_crtEncA = new CANcoder(19, Constants.kCanivoreBus);
    private final DutyCycleEncoder m_crtEncB = new DutyCycleEncoder(3);

    private final DigitalInput m_turretHomingHall = new DigitalInput(2);
    private final Trigger trg_homingHallDirect = new Trigger(homingEventLoop, () -> !m_turretHomingHall.get());
    private final BooleanLogger log_turretHomingHall = new BooleanLogger(kLogTab, "turretHomeHall");

    private final DoubleLogger log_crtEncAPos = WaltLogger.logDouble(kLogTab, "EncA/Pos");
    private final DoubleLogger log_crtEncBPos = WaltLogger.logDouble(kLogTab, "EncB/Pos");
    private final IntLogger log_crtEncBFreq = WaltLogger.logInt(kLogTab, "EncB/Freq");
    private final BooleanLogger log_crtEncBConn = WaltLogger.logBoolean(kLogTab, "EncB/Conn");

    private final DoubleLogger log_turretControlPos = WaltLogger.logDouble(kLogTab, "turretControlPos");
    private final DoubleLogger log_turretCRTPos = WaltLogger.logDouble(kLogTab, "turretCRTPos");
    

    public Turret() {
        m_turretMotor.getConfigurator().apply(kTurretTalonFXConfiguration);
        m_crtEncB.setAssumedFrequency(488);
        m_crtEncB.setConnectedFrequencyThreshold(400);
        // m_crtEncB.setDutyCycleRange(1/2049, 2048/2049);

        setDefaultCommand(m_homingCommand);

        double startAngle = calcTurretAngleCRT(m_crtEncA.getAbsolutePosition().getValueAsDouble() * 360, m_crtEncB.get() * 360); //shift into range of 0-360
        log_turretCRTPos.accept(startAngle);
        m_turretMotor.setPosition(kInitPosition);

        // trg_homingHallDirect.onTrue(Commands.sequence(
        //         Commands.runOnce(() -> {
        //             m_homingCommand.cancel();
        //             WaltLogger.timedPrint("FastHoming OK!!!");
        //         }),
        //         Commands.runOnce(() -> homingEventLoop.clear())));
    }

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

    public Command setTurretPosCmd(Angle rots) {
        return runOnce(() -> setTurretPos(rots));
    }

    public void setTurretPos(Angle rots) {
        setTurretPos(rots, RotationsPerSecond.zero());
    }

    public void setTurretPos(Angle rots, AngularVelocity velocityFF) {
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

    public Angle getCurrTurretPos() {
        return m_turretMotor.getPosition().getValue();
    }

    public boolean getTurretLocked() {
        return m_turretLocked;
    }

    public Angle getTurretLockAngle() {
        return m_turretLockAngle;
    }

    public boolean getHoldTurretAtIntake() {
        return m_holdTurretAtIntakePos;
    }

    public boolean isTurretHomed() {
        return m_isTurretHomed;
    }


    public void periodic() {
        log_crtEncAPos.accept(m_crtEncA.getAbsolutePosition().getValueAsDouble());

        log_crtEncBPos.accept(m_crtEncB.get());
        log_crtEncBFreq.accept(m_crtEncB.getFrequency());
        log_crtEncBConn.accept(m_crtEncB.isConnected());
    }

    public static double calcTurretAngleCRT(double e1, double e2) {
        double difference = e2 - e1;
        if (difference > 250) {
            difference -= 360;
        }
        if (difference < -250) {
            difference += 360;
        }
        difference *= kSlope;

        double e1Rotations = (difference * kGearZeroToothCount / kGearOneToothCount) / 360.0;
        double e1RotationsFloored = Math.floor(e1Rotations);
        double turretAngle = (e1RotationsFloored * 360.0 + e1) * (kGearOneToothCount / kGearZeroToothCount);
        if (turretAngle - difference < -100) {
            turretAngle += kGearOneToothCount / kGearZeroToothCount * 360.0;
        } else if (turretAngle - difference > 100) {
            turretAngle -= kGearOneToothCount / kGearZeroToothCount * 360.0;
        }

        return turretAngle;
    }

    public void fastPeriodic() {
        homingEventLoop.poll();
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
}
