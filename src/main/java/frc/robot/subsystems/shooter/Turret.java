package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import static frc.robot.Constants.TurretK.kLogTab;
import static frc.robot.Constants.TurretK.*;

import java.util.function.BooleanSupplier;
import frc.util.SignalManager;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.IntLogger;
import frc.util.WaltLogger.Pose3dLogger;

public class Turret extends SubsystemBase {
    private static final double kTurretMaxRotsFromHomeDeg = kTurretMaxRotsFromHome.in(Degrees);
    private boolean m_holdTurretAtIntakePos = false;
    private boolean m_turretLocked = false;
    private double m_turretLockAngleRots = 0.0;

    private final TalonFX m_turret = new TalonFX(kTurretCANID, Constants.kCanivoreBus); // X44Foc
    private final PositionVoltage m_PVRequest = new PositionVoltage(0).withEnableFOC(true);

    // ---LOGIC BOOLEANS
    private boolean m_isTurretHomed = true;
    public BooleanSupplier turretHomedSupp = () -> m_isTurretHomed;
    private boolean m_turretAtPos = false;
    public BooleanSupplier turretAtPosSupp = () -> m_turretAtPos;

    private final CANcoder m_lcmEncA = new CANcoder(19, Constants.kCanivoreBus);
    private final DutyCycleEncoder m_lcmEncB = new DutyCycleEncoder(3);

    private final DoubleLogger log_lcmEncAPos = WaltLogger.logDouble(kLogTab, "EncA/Pos");
    private final DoubleLogger log_lcmEncBPos = WaltLogger.logDouble(kLogTab, "EncB/Pos");
    private final IntLogger log_lcmEncBFreq = WaltLogger.logInt(kLogTab, "EncB/Freq");
    private final BooleanLogger log_lcmEncBConn = WaltLogger.logBoolean(kLogTab, "EncB/Conn");

    private final DoubleLogger log_turretControlPos = WaltLogger.logDouble(kLogTab, "turretControlPos");
    private final DoubleLogger log_turretLCMPos = WaltLogger.logDouble(kLogTab, "turretLCMPos");
    private final DoubleLogger log_turretClosedLoopError = WaltLogger.logDouble(kLogTab, "turretCLE");
    private final BooleanLogger log_atPos = WaltLogger.logBoolean(kLogTab, "atPos");
    private final BooleanLogger log_turretLocked = WaltLogger.logBoolean(kLogTab, "turretLocked");
    private final Pose3dLogger log_turretTransform = WaltLogger.logPose3d(kLogTab, "turretTransform");

    private final StatusSignal<Double> sig_turretCLErr = m_turret.getClosedLoopError();
    private final StatusSignal<Angle> sig_turretPos = m_turret.getPosition();
    private final StatusSignal<Angle> sig_lcmEncAAbsPos = m_lcmEncA.getAbsolutePosition();

    public Turret() {
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);
        m_lcmEncA.getConfigurator().apply(kEncoderAConfiguration);

        m_lcmEncB.setAssumedFrequency(488);
        m_lcmEncB.setConnectedFrequencyThreshold(400);

        SignalManager.register(Constants.kCanivoreBus, sig_turretCLErr, sig_turretPos, sig_lcmEncAAbsPos);

        log_turretTransform.accept(kTurretTransform);

        homeTurret(true);
    }

    public void homeTurret(boolean useLCM) {
        if (useLCM) {
            double lcmRots = calcTurretAngleLCM(sig_lcmEncAAbsPos.getValueAsDouble() * 360, -(m_lcmEncB.get() - kEncBOffset) * 360) / 360.0;
            m_turret.setPosition(lcmRots);
        } else {
            m_turret.setPosition(kInitPosition);
        }
    }

    private void refreshTurretAtPos() {
        log_turretClosedLoopError.accept(sig_turretCLErr.getValueAsDouble());
        m_turretAtPos = sig_turretCLErr.isNear(0, kTurretMaxErrD);
        log_atPos.accept(m_turretAtPos);
    }

    public boolean atPosition() {
        return m_turretAtPos;
    }

    public void setIntaking(boolean intaking) {
        m_holdTurretAtIntakePos = intaking;
    }

    public void setTurretLock(boolean locked) {
        m_turretLocked = locked;
        if (m_turretLocked) {
            m_turretLockAngleRots = sig_turretPos.getValueAsDouble();
        }

        log_turretLocked.accept(m_turretLocked);
    }

    public void lockAndSetTurretLockPos(double lockPos) {
        m_turretLocked = true;
        m_turretLockAngleRots = lockPos;

        log_turretLocked.accept(m_turretLocked);
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
        m_turret.setControl(m_PVRequest.withPosition(rots).withVelocity(velocityFF));
        log_turretControlPos.accept(rots.in(Rotations));
    }

    public void setTurretPos(double rots, double velocityFF) {
        m_turret.setControl(m_PVRequest.withPosition(rots).withVelocity(velocityFF));
        log_turretControlPos.accept(rots);
    }

    public void setTurretNeutralMode(NeutralModeValue value) {
        m_turret.setNeutralMode(value);
    }

    // for TestingDashboard
    public Command setTurretPositionCmd(DoubleSubscriber sub_rots) {
        return run(() -> setTurretPos(sub_rots.get(), 0.0));
    }

    public double getCurrTurretPos() {
        return sig_turretPos.getValueAsDouble();
    }

    public boolean getTurretLocked() {
        return m_turretLocked;
    }

    public double getTurretLockAngleRots() {
        return m_turretLockAngleRots;
    }

    public boolean getHoldTurretAtIntake() {
        return m_holdTurretAtIntakePos;
    }

    public boolean isTurretHomed() {
        return m_isTurretHomed;
    }


    public void periodic() {
        double encAVal = sig_lcmEncAAbsPos.getValueAsDouble();
        double encBVal = m_lcmEncB.get();
        log_lcmEncAPos.accept(encAVal);
        log_lcmEncBPos.accept(encBVal);
        log_lcmEncBFreq.accept(m_lcmEncB.getFrequency());
        log_lcmEncBConn.accept(m_lcmEncB.isConnected());

        refreshTurretAtPos();

        double turretAngleDeg = calcTurretAngleLCM(encAVal * 360, -(encBVal - kEncBOffset) * 360);
        log_turretLCMPos.accept(turretAngleDeg / 360.0);
    }

    public static double calcTurretAngleLCM(double e1, double e2) {
        double encARatio = kGearZeroToothCount / kGearOneToothCount; // 100/10.0
        double encBRatio = kGearZeroToothCount / kGearTwoToothCount; // 100/19
        double encAPeriod = 360.0 / encARatio;                       // 36° per turret rotation

        double bestAngle = 0;
        double bestError = Double.MAX_VALUE;

        // Center the search on the expected LCM output at home, spanning ±turret range
        double centerDeg = kLCMAtHomeRots * 360.0;
        double rangeDeg = kTurretMaxRotsFromHomeDeg;
        int kMin = (int) Math.floor((centerDeg - rangeDeg - e1 / encARatio) / encAPeriod);
        int kMax = (int) Math.ceil((centerDeg + rangeDeg - e1 / encARatio) / encAPeriod);

        for (int k = kMin; k <= kMax; k++) {
            double candidate = (e1 / encARatio) + (encAPeriod * k);

            double e2pred = (candidate * encBRatio) % 360.0;
            if (e2pred < 0) e2pred += 360.0;

            double error = e2pred - e2;
            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            if (Math.abs(error) < Math.abs(bestError)) {
                bestError = error;
                bestAngle = candidate;
            }
        }

        return bestAngle;
    }
}
