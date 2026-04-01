package frc.util;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;

public final class SignalManager {
    private static final Map<String, StatusSignalCollection> m_signalsByBus = new HashMap<>();

    private SignalManager() {}

    public static void register(String bus, BaseStatusSignal... signals) {
        m_signalsByBus.computeIfAbsent(bus, k -> new StatusSignalCollection()).addSignals(signals);
    }

    public static void register(CANBus bus, BaseStatusSignal... signals) {
        register(bus.getName(), signals);
    }

    public static void refreshAll() {
        for (var collection : m_signalsByBus.values()) {
            collection.refreshAll();
        }
    }
}
