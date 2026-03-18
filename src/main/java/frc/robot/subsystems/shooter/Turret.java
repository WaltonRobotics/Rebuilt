package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.IntLogger;

public class Turret {
    private final CANcoder m_crtEncA = new CANcoder(19, Constants.kCanivoreBus);
    private final DutyCycleEncoder m_crtEncB = new DutyCycleEncoder(3);

    private final DoubleLogger log_crtEncAPos = WaltLogger.logDouble("Turret", "EncA/Pos");
    private final DoubleLogger log_crtEncBPos = WaltLogger.logDouble("Turret", "EncB/Pos");
    private final IntLogger log_crtEncBFreq = WaltLogger.logInt("Turret", "EncB/Freq");
    private final BooleanLogger log_crtEncBConn = WaltLogger.logBoolean("Turret", "EncB/Conn");

    public Turret() {
        m_crtEncB.setAssumedFrequency(488);
        m_crtEncB.setConnectedFrequencyThreshold(400);
        // m_crtEncB.setDutyCycleRange(1/2049, 2048/2049);
    }

    public void periodic() {
        log_crtEncAPos.accept(m_crtEncA.getAbsolutePosition().getValueAsDouble());

        log_crtEncBPos.accept(m_crtEncB.get());
        log_crtEncBFreq.accept(m_crtEncB.getFrequency());
        log_crtEncBConn.accept(m_crtEncB.isConnected());
    }
}
