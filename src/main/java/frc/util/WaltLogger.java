package frc.util;

import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.IntConsumer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.struct.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class WaltLogger {
    private WaltLogger() {}
    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static final NetworkTable logTable = inst.getTable("Robot");

    public static void timedPrint(String label) {
        System.out.println("[" + Timer.getFPGATimestamp() + "] " + label);
    }

    public static Command timedPrintCmd(String label) {
        return Commands.runOnce(() -> timedPrint(label));
    }

    private static boolean shouldPublishNt() {
        // return false;
        return Constants.kDebugLoggingEnabled;
    }

    private static boolean dataLoggingEnabled() {
        return Constants.kDataLoggingEnabled;
    }

    /** Global toggle: true → enqueue to worker, false → publish synchronously on caller. */
    private static boolean shouldEnqueueAsync() {
        return Constants.kAsyncLoggingEnabled;
    }

    // ===== Async backend =====
    //
    // accept() on the caller thread captures the value + timestamp into a tiny
    // LogEvent lambda and offers it to a bounded queue. A single daemon worker
    // thread drains the queue and performs the actual ntPub.set / logEntry.append
    // calls. If the queue is ever full (worker stalled, sustained burst, GC),
    // accept() falls back to publishing inline so we never lose data.
    //
    // Timestamps are captured at accept() time via WPIUtilJNI.now() (microseconds,
    // same epoch as NT/DataLog) and passed through to the timestamped overloads
    // of set() / append(), so the published series reflects when each value was
    // actually produced — not when the worker happened to drain it.
    @FunctionalInterface
    private interface LogEvent { void publish(); }

    private static final int QUEUE_CAPACITY = 8192;
    private static final LinkedBlockingQueue<LogEvent> m_queue = new LinkedBlockingQueue<>(QUEUE_CAPACITY);
    private static final Thread m_worker;
    private static volatile boolean m_shuttingDown = false;

    // Cross-thread accept-time accounting. Every *Logger.accept(...) records its own
    // enqueue duration here; PerformanceMonitor drains this once per tick.
    // pub.set / logEntry.append paths going through StructPublishers outside this
    // class (e.g. WaltCamera.log_camPoseAndTag, ShooterCalc.pub_canTurretShoot) are
    // NOT tracked — only WaltLogger sinks are.
    private static final AtomicLong m_acceptNanosTotal = new AtomicLong();
    private static final AtomicLong m_acceptCallsTotal = new AtomicLong();

    // Worker-side accounting. Time and call count for the actual publish work.
    private static final AtomicLong m_publishNanosTotal = new AtomicLong();
    private static final AtomicLong m_publishCallsTotal = new AtomicLong();
    // Counts accept() calls that fell back to synchronous publish because the queue was full.
    private static final AtomicLong m_syncFallbackCalls = new AtomicLong();

    static {
        m_worker = new Thread(WaltLogger::workerLoop, "WaltLogger-Worker");
        m_worker.setDaemon(true);
        m_worker.start();
        Runtime.getRuntime().addShutdownHook(new Thread(WaltLogger::shutdown, "WaltLogger-Shutdown"));
    }

    private static void enqueue(LogEvent ev) {
        long s = System.nanoTime();
        if (!shouldEnqueueAsync()) {
            // Global toggle off: publish inline on the caller, bypass queue entirely.
            safePublish(ev);
        } else if (!m_queue.offer(ev)) {
            // Backpressure fallback: publish inline. Counted so it shows up in stats.
            m_syncFallbackCalls.incrementAndGet();
            safePublish(ev);
        }
        m_acceptNanosTotal.addAndGet(System.nanoTime() - s);
        m_acceptCallsTotal.incrementAndGet();
    }

    private static void safePublish(LogEvent ev) {
        long s = System.nanoTime();
        try {
            ev.publish();
        } catch (Throwable t) {
            // Never kill the worker (or fail the caller in fallback path) on a single bad publish.
        }
        m_publishNanosTotal.addAndGet(System.nanoTime() - s);
        m_publishCallsTotal.incrementAndGet();
    }

    private static void workerLoop() {
        while (true) {
            LogEvent ev;
            try {
                ev = m_queue.take();
            } catch (InterruptedException e) {
                // Shutdown signal: drain whatever's left and exit.
                int drained = 0;
                while ((ev = m_queue.poll()) != null) {
                    safePublish(ev);
                    drained++;
                }
                System.out.println("[WaltLogger] drained " + drained + " event(s) on shutdown");
                return;
            }
            safePublish(ev);
        }
    }

    /** Signal the worker to drain and exit. Idempotent; called from a JVM shutdown hook. */
    public static void shutdown() {
        if (m_shuttingDown) return;
        m_shuttingDown = true;
        m_worker.interrupt();
        try {
            m_worker.join(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public record AcceptStats(long totalNanos, long calls) {
        public double totalMs() { return totalNanos * 1e-6; }
    }

    public record PublishStats(long totalNanos, long calls, long syncFallbackCalls, int queueDepth) {
        public double totalMs() { return totalNanos * 1e-6; }
    }

    /** Read and reset the caller-side accept (enqueue) counters. Call once per tick. */
    public static AcceptStats drainAcceptStats() {
        return new AcceptStats(
            m_acceptNanosTotal.getAndSet(0L),
            m_acceptCallsTotal.getAndSet(0L));
    }

    /** Read and reset the worker-side publish counters; queueDepth is sampled instantaneously. */
    public static PublishStats drainPublishStats() {
        return new PublishStats(
            m_publishNanosTotal.getAndSet(0L),
            m_publishCallsTotal.getAndSet(0L),
            m_syncFallbackCalls.getAndSet(0L),
            m_queue.size());
    }

    public static BooleanEntry booleanItem(String table, String name, PubSubOption... options) {

        var topic = new BooleanTopic(NetworkTableInstance.getDefault().getTopic("Robot/" + table + "/" + name));

        return topic.getEntry(false, options);
    }

    public static StringEntry stringItem(String table, String name, String defaultVal, PubSubOption... options) {

        var topic = new StringTopic(NetworkTableInstance.getDefault().getTopic("Robot/" + table + "/" + name));

        return topic.getEntry(defaultVal, options);
    }

    public static IntLogger logInt(String table, String name, PubSubOption... options) {
        return new IntLogger(table, name, options);
    }

    public static final class Pose2dLogger implements Consumer<Pose2d> {
        public final StructPublisher<Pose2d> ntPub;
        public final StructLogEntry<Pose2d> logEntry;
        private final boolean alwaysPubNt;

        public Pose2dLogger(String subTable, String name, PubSubOption... options) {
            this(subTable, name, false, options);
        }

        public Pose2dLogger(String subTable, String name, boolean alwaysPubNt, PubSubOption... options) {
            StructTopic<Pose2d> topic = logTable.getSubTable(subTable).getStructTopic(name, new Pose2dStruct());
            this.alwaysPubNt = alwaysPubNt;
            ntPub = topic.publish(options);
            logEntry = StructLogEntry.create(DataLogManager.getLog(), "Robot/" + subTable + "/" + name, new Pose2dStruct());
        }

        @Override
        public void accept(Pose2d value) {
            final long ts = WPIUtilJNI.now();
            enqueue(() -> {
                if (shouldPublishNt()) {
                    ntPub.set(value, ts);
                } else {
                    if (alwaysPubNt) ntPub.set(value, ts);
                    if (dataLoggingEnabled()) logEntry.append(value, ts);
                }
            });
        }

        public void accept(Translation2d value) {
            accept(new Pose2d(value, Rotation2d.kZero));
        }

        public void accept(Translation2d translation, Rotation2d value) {
            accept(new Pose2d(translation, value));
        }
    }

    public static Pose2dLogger logPose2d(String table, String name, boolean alwaysPubNt, PubSubOption... options) {
        return new Pose2dLogger(table, name, alwaysPubNt, options);
    }

    public static final class Pose2dArrayLogger implements Consumer<Pose2d[]> {
        public final StructArrayPublisher<Pose2d> ntPub;
        public final StructArrayLogEntry<Pose2d> logEntry;

        public Pose2dArrayLogger(String subTable, String name, PubSubOption... options) {
            StructArrayTopic<Pose2d> topic = logTable.getSubTable(subTable).getStructArrayTopic(name, new Pose2dStruct());
            ntPub = topic.publish(options);
            logEntry = StructArrayLogEntry.create(DataLogManager.getLog(), "Robot/" + subTable + "/" + name, new Pose2dStruct());
        }

        @Override
        public void accept(Pose2d[] value) {
            final long ts = WPIUtilJNI.now();
            // Defensive shallow copy: caller may mutate `value` before the worker drains it.
            // Pose2d itself is immutable, so a shallow clone is sufficient.
            final Pose2d[] copy = value.clone();
            enqueue(() -> {
                if (shouldPublishNt()) ntPub.set(copy, ts);
                else if (dataLoggingEnabled()) logEntry.append(copy, ts);
            });
        }

        public void accept(Translation2d[] translations) {
            Pose2d[] poses = new Pose2d[translations.length];
            for (int i = 0; i < translations.length; i++) {
                poses[i] = new Pose2d(translations[i], Rotation2d.kZero);
            }
            accept(poses);
        }

        public void accept(Translation2d[] translations, Rotation2d[] rotations) {
            Pose2d[] poses = new Pose2d[translations.length];
            for (int i = 0; i < translations.length; i++) {
                poses[i] = new Pose2d(translations[i], rotations[i]);
            }
            accept(poses);
        }
    }

    public static Pose2dArrayLogger logPose2dArray(String table, String name, PubSubOption... options) {
        return new Pose2dArrayLogger(table, name, options);
    }

    public static final class Pose3dLogger implements Consumer<Pose3d> {
        public final StructPublisher<Pose3d> ntPub;
        public final StructLogEntry<Pose3d> logEntry;

        public Pose3dLogger(String subTable, String name, PubSubOption... options) {
            StructTopic<Pose3d> topic = logTable.getSubTable(subTable).getStructTopic(name, new Pose3dStruct());
            ntPub = topic.publish(options);
            logEntry = StructLogEntry.create(DataLogManager.getLog(), "Robot/" + subTable + "/" + name, new Pose3dStruct());
        }

        @Override
        public void accept(Pose3d value) {
            final long ts = WPIUtilJNI.now();
            enqueue(() -> {
                if (shouldPublishNt()) ntPub.set(value, ts);
                else if (dataLoggingEnabled()) logEntry.append(value, ts);
            });
        }

        public void accept(Translation3d value) {
            accept(new Pose3d(value.getX(), value.getY(), value.getZ(), new Rotation3d()));
        }

        public void accept(Transform3d value) {
            accept(new Pose3d(value.getTranslation(), value.getRotation()));
        }
    }

    public static Pose3dLogger logPose3d(String name, String table, PubSubOption... options) {
        return new Pose3dLogger(name, table, options);
    }

    public static final class Translation3dArrayLogger implements Consumer<Translation3d[]> {
        public final StructArrayPublisher<Translation3d> ntPub;
        public final StructArrayLogEntry<Translation3d> logEntry;

        public Translation3dArrayLogger(String subTable, String name, PubSubOption... options) {
            StructArrayTopic<Translation3d> topic = logTable.getSubTable(subTable).getStructArrayTopic(name, new Translation3dStruct());
            ntPub = topic.publish(options);
            logEntry = StructArrayLogEntry.create(DataLogManager.getLog(), "Robot/" + subTable + "/" + name, new Translation3dStruct());
        }

        @Override
        public void accept(Translation3d[] value) {
            final long ts = WPIUtilJNI.now();
            final Translation3d[] copy = value.clone();
            enqueue(() -> {
                if (shouldPublishNt()) ntPub.set(copy, ts);
                else if (dataLoggingEnabled()) logEntry.append(copy, ts);
            });
        }
    }

    public static Translation3dArrayLogger logTranslation3dArray(String table, String name, PubSubOption... options) {
        return new Translation3dArrayLogger(table, name, options);
    }

    // public static final class Translation2dLogger implements Consumer<Translation2d> {
    //     public final StructPublisher<Translation2d> ntPub;
    //     public final StructLogEntry<Translation2d> logEntry;

    //     public Translation2dLogger(String subTable, String name, PubSubOption... options) {
    //         StructTopic<Translation2d> topic = logTable.getSubTable(subTable).getStructTopic(name, new Translation2dStruct());
    //         ntPub = topic.publish(options);
    //         logEntry = StructLogEntry.create(DataLogManager.getLog(), "Robot/" + subTable + "/" + name, new Translation2dStruct());
    //     }

    //     @Override
    //     public void accept(Translation2d value) {
    //         if (shouldPublishNt()) {
    //             ntPub.set(value);
    //         } else {
    //             logEntry.append(value);
    //         }
    //     }
    // }

    // public static Translation2dLogger logTranslation2d(String table, String name, PubSubOption... options) {
    //     return new Translation2dLogger(table, name, options);
    // }

    public static final class IntLogger implements IntConsumer {
        public final IntegerPublisher ntPub;
        public final IntegerLogEntry logEntry;

        public IntLogger(String subTable, String name, PubSubOption... options) {
            ntPub = NTPublisherFactory.makeIntPub(logTable.getSubTable(subTable), name, options);
            logEntry = new IntegerLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
        }

        @Override
        public void accept(int value) {
            final long ts = WPIUtilJNI.now();
            final long v = value;
            enqueue(() -> {
                if (shouldPublishNt()) ntPub.set(v, ts);
                else if (dataLoggingEnabled()) logEntry.append(v, ts);
            });
        }
    }

    public static final class DoubleLogger implements DoubleConsumer {
        public final DoublePublisher ntPub;
        public final DoubleLogEntry logEntry;

        public DoubleLogger(String subTable, String name, PubSubOption... options) {
            ntPub = NTPublisherFactory.makeDoublePub(logTable.getSubTable(subTable), name, options);
            logEntry = new DoubleLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
        }

        @Override
        public void accept(double value) {
            final long ts = WPIUtilJNI.now();
            enqueue(() -> {
                if (shouldPublishNt()) ntPub.set(value, ts);
                else if (dataLoggingEnabled()) logEntry.append(value, ts);
            });
        }
    }

    public static DoubleLogger logDouble(String table, String name, PubSubOption... options) {
        return new DoubleLogger(table, name, options);
    }

    public static final class BooleanLogger implements BooleanConsumer {
        public final BooleanPublisher ntPub;
        public final BooleanLogEntry logEntry;

        public BooleanLogger(String subTable, String name, PubSubOption... options) {
            ntPub = NTPublisherFactory.makeBoolPub(logTable.getSubTable(subTable), name, options);
            logEntry = new BooleanLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
        }

        @Override
        public void accept(boolean value) {
            final long ts = WPIUtilJNI.now();
            enqueue(() -> {
                if (shouldPublishNt()) ntPub.set(value, ts);
                else if (dataLoggingEnabled()) logEntry.append(value, ts);
            });
        }

        public void accept(BooleanSupplier valueSup) {
            accept(valueSup.getAsBoolean());
        }
    }

    public static BooleanLogger logBoolean(String table, String name, PubSubOption... options) {
        return new BooleanLogger(table, name, options);
    }

    public static final class DoubleArrayLogger implements Consumer<double[]> {
        public final DoubleArrayPublisher ntPub;
        public final DoubleArrayLogEntry logEntry;

        public DoubleArrayLogger(String subTable, String name) {
            ntPub = NTPublisherFactory.makeDoubleArrPub(logTable.getSubTable(subTable), name);
            logEntry = new DoubleArrayLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
        }

        @Override
        public void accept(double[] value) {
            final long ts = WPIUtilJNI.now();
            final double[] copy = value.clone();
            enqueue(() -> {
                if (shouldPublishNt()) ntPub.set(copy, ts);
                else if (dataLoggingEnabled()) logEntry.append(copy, ts);
            });
        }
    }

    public static DoubleArrayLogger logDoubleArray(String table, String name) {
        return new DoubleArrayLogger(table, name);
    }

    public static final class StringLogger implements Consumer<String> {
        public final StringPublisher ntPub;
        public final StringLogEntry logEntry;

        public StringLogger(String subTable, String name, PubSubOption... options) {
            ntPub = NTPublisherFactory.makeStringPub(logTable.getSubTable(subTable), name, options);
            logEntry = new StringLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
        }

        @Override
        public void accept(String value) {
            final long ts = WPIUtilJNI.now();
            enqueue(() -> {
                if (shouldPublishNt()) ntPub.set(value, ts);
                else if (dataLoggingEnabled()) logEntry.append(value, ts);
            });
        }
    }

    public static StringLogger logString(String table, String name, PubSubOption... options) {
        return new StringLogger(table, name, options);
    }

    public static final class StringArrayLogger implements Consumer<String[]> {
        public final StringArrayPublisher ntPub;
        public final StringArrayLogEntry logEntry;

        public StringArrayLogger(String subTable, String name, PubSubOption... options) {
            ntPub = NTPublisherFactory.makeStringArrPub(logTable.getSubTable(subTable), name, options);
            logEntry = new StringArrayLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
        }

        @Override
        public void accept(String[] value) {
            final long ts = WPIUtilJNI.now();
            final String[] copy = value.clone();
            enqueue(() -> {
                if (shouldPublishNt()) ntPub.set(copy, ts);
                else if (dataLoggingEnabled()) logEntry.append(copy, ts);
            });
        }
    }

    public static StringArrayLogger logStringArray(String table, String name, PubSubOption... options) {
        return new StringArrayLogger(table, name, options);
    }
}
