package frc.util;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;

import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.IntLogger;

public final class SignalManager {
    private static final Map<CANBus, StatusSignalCollection> m_signalsByBus = new HashMap<>();
    private static final Map<CANBus, BusLoggers> m_busLoggers = new HashMap<>();

    private SignalManager() {}

    private static final class BusLoggers {
        final DoubleLogger busUtilization;
        final IntLogger busOffCount;
        final IntLogger txFullCount;
        final IntLogger rec;
        final IntLogger tec;
        final BooleanLogger isValid;

        BusLoggers(String busName) {
            String tab = "CANBusStatus/" + busName;
            busUtilization = WaltLogger.logDouble(tab, "BusUtilization");
            busOffCount = WaltLogger.logInt(tab, "BusOffCount");
            txFullCount = WaltLogger.logInt(tab, "TxFullCount");
            rec = WaltLogger.logInt(tab, "REC");
            tec = WaltLogger.logInt(tab, "TEC");
            isValid = WaltLogger.logBoolean(tab, "IsValid");
        }

        void log(CANBus.CANBusStatus status) {
            busUtilization.accept(status.BusUtilization);
            busOffCount.accept(status.BusOffCount);
            txFullCount.accept(status.TxFullCount);
            rec.accept(status.REC);
            tec.accept(status.TEC);
            isValid.accept(status.Status.isOK());
        }
    }

    public static void register(CANBus bus, BaseStatusSignal... signals) {
        m_signalsByBus.computeIfAbsent(bus, k -> new StatusSignalCollection()).addSignals(signals);
        m_busLoggers.computeIfAbsent(bus, k -> new BusLoggers(bus.getName()));
    }

    public static void refreshAll() {
        for (var entry : m_signalsByBus.entrySet()) {
            entry.getValue().refreshAll();
        }

        for (var entry : m_busLoggers.entrySet()) {
            entry.getValue().log(entry.getKey().getStatus());
        }
    }
}
