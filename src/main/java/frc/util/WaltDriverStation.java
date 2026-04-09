package frc.util;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

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
