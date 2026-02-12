package frc.util;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.TimedRobot;

public final class NTPublisherFactory {
    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static final NetworkTable traceTable = inst.getTable("Trace");

    static public IntegerPublisher makeIntPub(NetworkTable table, String name, PubSubOption... options) {
        IntegerTopic topic = table.getIntegerTopic(name);
        return topic.publish(options);
    }

    static public IntegerPublisher makeIntPub(String table, String name, PubSubOption... options) {
        return makeIntPub(inst.getTable(table), name, options);
    }

    static public DoublePublisher makeDoublePub(NetworkTable table, String name, PubSubOption... options) {
        DoubleTopic topic = table.getDoubleTopic(name);
        return topic.publish(options);
    }

    static public DoublePublisher makeDoublePub(String table, String name, PubSubOption... options) {
        return makeDoublePub(inst.getTable(table), name, options);
    }

    static public DoublePublisher makeDoublePub(String name) {
        return makeDoublePub(traceTable, name, PubSubOption.periodic(TimedRobot.kDefaultPeriod));
    }

    static public DoubleArrayPublisher makeDoubleArrPub(NetworkTable table, String name, PubSubOption... options) {
        DoubleArrayTopic topic = table.getDoubleArrayTopic(name);
        return topic.publish(options);
    }

    static public DoubleArrayPublisher makeDoubleArrPub(String table, String name, PubSubOption... options) {
        return makeDoubleArrPub(inst.getTable(table), name, options);
    }

    static public DoubleArrayPublisher makeDoubleArrTracePub(String name) {
        return makeDoubleArrPub(traceTable, name, PubSubOption.periodic(TimedRobot.kDefaultPeriod));
    }

    static public BooleanPublisher makeBoolPub(NetworkTable table, String name, PubSubOption... options) {
        BooleanTopic topic = table.getBooleanTopic(name);
        return topic.publish(options);
    }

    static public BooleanPublisher makeBoolPub(String table, String name, PubSubOption... options) {
        return makeBoolPub(inst.getTable(table), name, options);
    }

    static public BooleanPublisher makeBoolTracePub(String name) {
        return makeBoolPub(traceTable, name, PubSubOption.periodic(TimedRobot.kDefaultPeriod));
    }

    static public StringPublisher makeStringPub(NetworkTable table, String name, PubSubOption... options) {
        StringTopic topic = table.getStringTopic(name);
        return topic.publish(options);
    }

    static public StringPublisher makeStringPub(String table, String name, PubSubOption... options) {
        return makeStringPub(inst.getTable(table), name, options);
    }

    static public StringPublisher makeStringTracePub(String name) {
        return makeStringPub(traceTable, name, PubSubOption.periodic(TimedRobot.kDefaultPeriod));
    }
}