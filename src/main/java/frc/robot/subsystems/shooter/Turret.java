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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterK.*;
import static frc.robot.Constants.TurretK.*;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.IntLogger;

public class Turret extends SubsystemBase {
    private boolean m_holdTurretAtIntakePos = false;
    private boolean m_turretLocked = false;
    private Angle m_turretLockAngle = Degrees.zero();

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

    private final CANcoder m_crtEncA = new CANcoder(19, Constants.kCanivoreBus);
    private final DutyCycleEncoder m_crtEncB = new DutyCycleEncoder(3);

    private final DoubleLogger log_crtEncAPos = WaltLogger.logDouble("Turret", "EncA/Pos");
    private final DoubleLogger log_crtEncBPos = WaltLogger.logDouble("Turret", "EncB/Pos");
    private final IntLogger log_crtEncBFreq = WaltLogger.logInt("Turret", "EncB/Freq");
    private final BooleanLogger log_crtEncBConn = WaltLogger.logBoolean("Turret", "EncB/Conn");

    private final DoubleLogger log_turretControlPos = WaltLogger.logDouble("Shooter/Turret", "turretControlPos");

    public Turret() {
        m_turretMotor.getConfigurator().apply(kTurretTalonFXConfiguration);
        m_crtEncB.setAssumedFrequency(488);
        m_crtEncB.setConnectedFrequencyThreshold(400);
        // m_crtEncB.setDutyCycleRange(1/2049, 2048/2049);

        double startAngle = calcTurretAngleCRT(m_crtEncA.getAbsolutePosition().getValueAsDouble(), m_crtEncB.get());
        m_turretMotor.setPosition(startAngle);

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
}
