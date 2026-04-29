package frc.util;

import java.lang.management.ClassLoadingMXBean;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;
import java.lang.management.ThreadMXBean;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javax.management.Notification;
import javax.management.NotificationEmitter;
import javax.management.NotificationListener;

import com.sun.management.GarbageCollectionNotificationInfo;
import com.sun.management.GcInfo;
import frc.util.WaltLogger.DoubleLogger;

public class PerformanceMonitor {
    private static final String kLogTab = "Perf";

    private final DoubleLogger log_loopTimeMs = WaltLogger.logDouble(kLogTab, "loopTimeMs");
    // CPU time the main thread actually spent executing this tick. Compare to loopTimeMs:
    //   - both spike together  → the loop is doing more work
    //   - wall spikes, cpu flat → the loop is being preempted (other threads, kernel, STW GC)
    private final DoubleLogger log_loopCpuMs = WaltLogger.logDouble(kLogTab, "loopCpuMs");
    private final DoubleLogger log_loopBlockedMs = WaltLogger.logDouble(kLogTab, "loopBlockedMs");
    // Cross-thread WaltLogger accounting, drained once per tick.
    //
    // loggerMs / loggerCalls = caller-side enqueue cost. With the async backend this
    //   is what the main loop (and any other caller thread) actually pays — should be
    //   ~0 in steady state. Captures all WaltLogger.*Logger.accept(...) calls from any
    //   thread (main loop, ShooterCalc notifier, sim notifier, swerve odometry).
    //   Direct StructPublisher .set() calls outside WaltLogger (e.g. log_camPoseAndTag)
    //   are NOT included.
    // publishMs / publishCalls = worker-thread cost of the actual ntPub.set / logEntry.append.
    //   Lives off the main loop; useful for spotting publish-side hotspots.
    // syncFallbackCalls = accept() calls that bypassed the queue because it was full and
    //   published inline. Should stay at 0; non-zero means worker is starved or stalled.
    // queueDepth = instantaneous queue depth at end of tick. Should stay near 0.
    private final DoubleLogger log_loggerMs = WaltLogger.logDouble(kLogTab, "loggerMs");
    private final DoubleLogger log_loggerCalls = WaltLogger.logDouble(kLogTab, "loggerCalls");
    private final DoubleLogger log_publishMs = WaltLogger.logDouble(kLogTab, "publishMs");
    private final DoubleLogger log_publishCalls = WaltLogger.logDouble(kLogTab, "publishCalls");
    private final DoubleLogger log_syncFallbackCalls = WaltLogger.logDouble(kLogTab, "syncFallbackCalls");
    private final DoubleLogger log_queueDepth = WaltLogger.logDouble(kLogTab, "queueDepth");
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
    private long m_loopStartCpuNanos;
    private final MemoryMXBean m_memBean;
    private final ThreadMXBean m_threadBean;
    private final boolean m_cpuTimeSupported;
    private final List<GarbageCollectorMXBean> m_gcBeans;
    private long m_prevGcCount;
    private long m_prevGcTimeMs;

    private record GcEvent(String cause, String action, long durationMs) {}
    private final AtomicReference<GcEvent> m_lastGcEvent = new AtomicReference<>();

    private static final double BYTES_TO_MB = 1.0 / (1024.0 * 1024.0);

    public PerformanceMonitor(boolean trackClassLoading) {
        m_trackClassLoading = trackClassLoading;
        m_memBean = ManagementFactory.getMemoryMXBean();
        m_threadBean = ManagementFactory.getThreadMXBean();
        m_cpuTimeSupported = m_threadBean.isCurrentThreadCpuTimeSupported();
        if (m_cpuTimeSupported && !m_threadBean.isThreadCpuTimeEnabled()) {
            m_threadBean.setThreadCpuTimeEnabled(true);
        }
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
        m_loopStartCpuNanos = m_cpuTimeSupported ? m_threadBean.getCurrentThreadCpuTime() : 0L;
    }

    /** Call at the very end of robotPeriodic(). Logs loop time, CPU, and memory usage. */
    public void loopEnd() {
        long now = System.nanoTime();
        double wallMs = (now - m_loopStartNanos) * 1e-6;
        log_loopTimeMs.accept(wallMs);

        if (m_cpuTimeSupported) {
            double cpuMs = (m_threadBean.getCurrentThreadCpuTime() - m_loopStartCpuNanos) * 1e-6;
            log_loopCpuMs.accept(cpuMs);
            log_loopBlockedMs.accept(Math.max(0.0, wallMs - cpuMs));
        }

        // Drain logger stats. This deliberately runs AFTER all the per-section
        // logging above so it captures their cost too.
        var acceptStats = WaltLogger.drainAcceptStats();
        log_loggerMs.accept(acceptStats.totalMs());
        log_loggerCalls.accept(acceptStats.calls());
        var publishStats = WaltLogger.drainPublishStats();
        log_publishMs.accept(publishStats.totalMs());
        log_publishCalls.accept(publishStats.calls());
        log_syncFallbackCalls.accept(publishStats.syncFallbackCalls());
        log_queueDepth.accept(publishStats.queueDepth());

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
    }
}
