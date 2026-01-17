package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final boolean kDebugLoggingEnabled = true;

    public static class FieldK {
        public static final double kFieldLengthMeters = Units.inchesToMeters(651.22); //take with a grain of salt - pulled from field dimensions (welded)
        public static final double kFieldWidthMeters = Units.inchesToMeters(317.69);
    }

    public static class RobotK {
        public static final String kLogTab = "Superstructure";
    }
}
