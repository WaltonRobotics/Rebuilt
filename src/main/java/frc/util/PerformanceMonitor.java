package frc.util;

import java.io.IOException;
import java.lang.management.ClassLoadingMXBean;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javax.management.Notification;
import javax.management.NotificationEmitter;
import javax.management.NotificationListener;

import com.sun.management.GarbageCollectionNotificationInfo;
import com.sun.management.GcInfo;
import org.wpilib.framework.RobotBase;
import frc.util.WaltLogger.DoubleLogger;

public class PerformanceMonitor {
    private static final String kLogTab = "Perf";

    private final DoubleLogger log_loopTimeMs = WaltLogger.logDouble(kLogTab, "loopTimeMs");
    private final DoubleLogger log_heapUsedMB = WaltLogger.logDouble(kLogTab, "heapUsedMB");
    private final DoubleLogger log_heapMaxMB = WaltLogger.logDouble(kLogTab, "heapMaxMB");
    private final DoubleLogger log_heapCommittedMB = WaltLogger.logDouble(kLogTab, "heapCommittedMB");
    private final DoubleLogger log_nonHeapUsedMB = WaltLogger.logDouble(kLogTab, "nonHeapUsedMB");
    private final DoubleLogger log_gcCountDelta = WaltLogger.logDouble(kLogTab, "gcCountDelta");
    private final DoubleLogger log_gcTimeMs = WaltLogger.logDouble(kLogTab, "gcTimeMs");
    private final DoubleLogger log_gcLastPauseMs = WaltLogger.logDouble(kLogTab, "gcLastPauseMs");
    private final WaltLogger.StringLogger log_gcLastCause = WaltLogger.logString(kLogTab, "gcLastCause");

    private final boolean m_trackClassLoading;
    private DoubleLogger log_classesLoadedDelta;
    private DoubleLogger log_classesLoadedTotal;
    private ClassLoadingMXBean m_classBean;
    private long m_prevClassesLoaded;

    private long m_loopStartNanos;
    private final MemoryMXBean m_memBean;
    private final List<GarbageCollectorMXBean> m_gcBeans;
    private long m_prevGcCount;
    private long m_prevGcTimeMs;

    private record GcEvent(String cause, String action, long durationMs) {}
    private final AtomicReference<GcEvent> m_lastGcEvent = new AtomicReference<>();

    private static final double BYTES_TO_MB = 1.0 / (1024.0 * 1024.0);

    // Disk I/O via /proc/self/io (Linux only). Track block-device-actual write_bytes,
    // user-space wchar (covers buffered writes that haven't hit disk yet), and
    // syscall counts so we can attribute spikes to flush events vs steady traffic.
    private static final Path kProcSelfIo = Path.of("/proc/self/io");
    private boolean m_diskIoAvailable = RobotBase.isReal();
    private long m_prevWriteBytes;
    private long m_prevWcharBytes;
    private long m_prevWriteSyscalls;
    private long m_prevReadBytes;
    private final DoubleLogger log_diskWriteBytesDelta = WaltLogger.logDouble(kLogTab, "diskWriteBytesDelta");
    private final DoubleLogger log_diskWcharBytesDelta = WaltLogger.logDouble(kLogTab, "diskWcharBytesDelta");
    private final DoubleLogger log_diskWriteSyscallsDelta = WaltLogger.logDouble(kLogTab, "diskWriteSyscallsDelta");
    private final DoubleLogger log_diskReadBytesDelta = WaltLogger.logDouble(kLogTab, "diskReadBytesDelta");

    public PerformanceMonitor(boolean trackClassLoading) {
        m_trackClassLoading = trackClassLoading;
        m_memBean = ManagementFactory.getMemoryMXBean();
        m_gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
        m_prevGcCount = totalGcCount();
        m_prevGcTimeMs = totalGcTimeMs();

        NotificationListener gcListener = (Notification notif, Object handback) -> {
            if (notif.getType().equals(GarbageCollectionNotificationInfo.GARBAGE_COLLECTION_NOTIFICATION)) {
                var gcNotifInfo = GarbageCollectionNotificationInfo.from(
                    (javax.management.openmbean.CompositeData) notif.getUserData());
                GcInfo gcInfo = gcNotifInfo.getGcInfo();
                m_lastGcEvent.set(new GcEvent(
                    gcNotifInfo.getGcCause(),
                    gcNotifInfo.getGcAction(),
                    gcInfo.getDuration()));
            }
        };
        for (var gc : m_gcBeans) {
            ((NotificationEmitter) gc).addNotificationListener(gcListener, null, null);
        }

        if (m_trackClassLoading) {
            m_classBean = ManagementFactory.getClassLoadingMXBean();
            log_classesLoadedDelta = WaltLogger.logDouble(kLogTab, "classesLoadedDelta");
            log_classesLoadedTotal = WaltLogger.logDouble(kLogTab, "classesLoadedTotal");
            m_prevClassesLoaded = m_classBean.getTotalLoadedClassCount();
        }

        // Prime baseline counters so the first delta isn't an artifact of startup I/O.
        if (m_diskIoAvailable) sampleDiskIo(false);
    }

    /**
     * Reads /proc/self/io and either primes baselines (logDelta=false) or logs the
     * delta since the last sample. Sets m_diskIoAvailable=false on first failure.
     */
    private void sampleDiskIo(boolean logDelta) {
        try {
            long writeBytes = 0, wcharBytes = 0, writeSyscalls = 0, readBytes = 0;
            for (String line : Files.readAllLines(kProcSelfIo)) {
                int colon = line.indexOf(':');
                if (colon < 0) continue;
                String key = line.substring(0, colon);
                long value = Long.parseLong(line.substring(colon + 1).trim());
                switch (key) {
                    case "write_bytes" -> writeBytes = value;
                    case "wchar"       -> wcharBytes = value;
                    case "syscw"       -> writeSyscalls = value;
                    case "read_bytes"  -> readBytes = value;
                    default            -> {}
                }
            }
            if (logDelta) {
                log_diskWriteBytesDelta.accept((double) (writeBytes - m_prevWriteBytes));
                log_diskWcharBytesDelta.accept((double) (wcharBytes - m_prevWcharBytes));
                log_diskWriteSyscallsDelta.accept((double) (writeSyscalls - m_prevWriteSyscalls));
                log_diskReadBytesDelta.accept((double) (readBytes - m_prevReadBytes));
            }
            m_prevWriteBytes = writeBytes;
            m_prevWcharBytes = wcharBytes;
            m_prevWriteSyscalls = writeSyscalls;
            m_prevReadBytes = readBytes;
        } catch (IOException | NumberFormatException e) {
            m_diskIoAvailable = false;
        }
    }

    private long totalGcCount() {
        long total = 0;
        for (var gc : m_gcBeans) total += gc.getCollectionCount();
        return total;
    }

    private long totalGcTimeMs() {
        long total = 0;
        for (var gc : m_gcBeans) total += gc.getCollectionTime();
        return total;
    }

    /** Call at the very start of robotPeriodic(). */
    public void loopStart() {
        m_loopStartNanos = System.nanoTime();
    }

    /** Call at the very end of robotPeriodic(). Logs loop time, CPU, and memory usage. */
    public void loopEnd() {
        long now = System.nanoTime();
        log_loopTimeMs.accept((now - m_loopStartNanos) * 1e-6);

        MemoryUsage heap = m_memBean.getHeapMemoryUsage();
        long heapMax = heap.getMax();
        log_heapUsedMB.accept(heap.getUsed() * BYTES_TO_MB);
        log_heapMaxMB.accept(heapMax < 0 ? Double.NaN : heapMax * BYTES_TO_MB);
        log_heapCommittedMB.accept(heap.getCommitted() * BYTES_TO_MB);
        log_nonHeapUsedMB.accept(m_memBean.getNonHeapMemoryUsage().getUsed() * BYTES_TO_MB);

        long gcCount = totalGcCount();
        long gcTimeMs = totalGcTimeMs();
        log_gcCountDelta.accept((double) (gcCount - m_prevGcCount));
        log_gcTimeMs.accept((double) (gcTimeMs - m_prevGcTimeMs));
        m_prevGcCount = gcCount;
        m_prevGcTimeMs = gcTimeMs;

        GcEvent evt = m_lastGcEvent.getAndSet(null);
        if (evt != null) {
            log_gcLastPauseMs.accept((double) evt.durationMs());
            log_gcLastCause.accept(evt.action() + ": " + evt.cause());
        }

        if (m_trackClassLoading) {
            long totalLoaded = m_classBean.getTotalLoadedClassCount();
            log_classesLoadedDelta.accept((double) (totalLoaded - m_prevClassesLoaded));
            log_classesLoadedTotal.accept((double) totalLoaded);
            m_prevClassesLoaded = totalLoaded;
        }

        if (m_diskIoAvailable) sampleDiskIo(true);
    }
}
