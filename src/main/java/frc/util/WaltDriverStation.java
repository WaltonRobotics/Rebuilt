package frc.util;

import java.util.Optional;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStation.Alliance;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.RobotModeTriggers;

public final class WaltDriverStation {
    private static Optional<Alliance> m_cachedAlliance = Optional.empty();

    static {
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(WaltDriverStation::cacheAlliance));
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(WaltDriverStation::cacheAlliance));
    }

    private WaltDriverStation() {}

    static void cacheAlliance() {
        DriverStation.getAlliance().ifPresent(alliance -> m_cachedAlliance = Optional.of(alliance));
    }

    /** Returns the alliance cached at the start of the current/last autonomous or teleop period. */
    public static Optional<Alliance> getAlliance() {
        return m_cachedAlliance;
    }
}
