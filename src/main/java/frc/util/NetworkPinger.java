package frc.util;

import java.net.InetAddress;

import org.wpilib.system.Notifier;
import frc.util.WaltLogger.BooleanLogger;

public class NetworkPinger {
    private final InetAddress m_address;
    private final BooleanLogger m_log;
    private final Notifier m_notifier;
    private final int m_timeoutMs;

    public NetworkPinger(String name, String ip, double periodSecs, int timeoutMs) {
        m_timeoutMs = timeoutMs;
        m_log = WaltLogger.logBoolean("Network", name + "/reachable");

        try {
            m_address = InetAddress.getByName(ip);
        } catch (Exception e) {
            throw new RuntimeException("Invalid IP for NetworkPinger: " + ip, e);
        }

        m_notifier = new Notifier(this::ping);
        m_notifier.setName("NetworkPinger-" + name);
        m_notifier.startPeriodic(periodSecs);
    }

    private void ping() {
        try {
            boolean reachable = m_address.isReachable(m_timeoutMs);
            m_log.accept(reachable);
        } catch (Exception e) {
            m_log.accept(false);
        }
    }

}
