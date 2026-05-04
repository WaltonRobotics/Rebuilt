// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.util;

import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStation.Alliance;
import org.wpilib.system.Timer;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import frc.robot.subsystems.shooter.ShotCalculator;

public class HubShiftUtil {
  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED;
  }

  public record ShiftInfo(
      ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

  private static Timer shiftTimer = new Timer();
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  private static final double minFuelCountDelay = 1.0;
  private static final double maxFuelCountDelay = 2.0;
  private static final double shiftEndFuelCountExtension = 3.0;
  private static final double minTimeOfFlight = ShotCalculator.getMinTimeOfFlight();
  private static final double maxTimeOfFlight = ShotCalculator.getMaxTimeOfFlight();
  private static final double approachingActiveFudge = -1 * (minTimeOfFlight + minFuelCountDelay);
  private static final double endingActiveFudge =
      shiftEndFuelCountExtension + -1 * (maxTimeOfFlight + maxFuelCountDelay);
  private static final double approachingActiveComeback = approachingActiveFudge - 3.0;
  private static final double endingActiveComeback = endingActiveFudge + 3.0;

  public static final double autoEndTime = 20.0;
  public static final double teleopDuration = 140.0;
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

  // Pre-computed shifted time arrays (all values are constants, no need to reallocate each call)
  private static final double[] kShiftedStartActive = {0.0, 10.0, 35.0 + endingActiveFudge, 60.0 + approachingActiveFudge, 85.0 + endingActiveFudge, 110.0 + approachingActiveFudge};
  private static final double[] kShiftedEndActive = {10.0, 35.0 + endingActiveFudge, 60.0 + approachingActiveFudge, 85.0 + endingActiveFudge, 110.0 + approachingActiveFudge, 140.0};
  private static final double[] kShiftedStartInactive = {0.0, 10.0 + endingActiveFudge, 35.0 + approachingActiveFudge, 60.0 + endingActiveFudge, 85.0 + approachingActiveFudge, 110.0};
  private static final double[] kShiftedEndInactive = {10.0 + endingActiveFudge, 35.0 + approachingActiveFudge, 60.0 + endingActiveFudge, 85.0 + approachingActiveFudge, 110.0, 140.0};
  private static final double[] kComebackStartActive = {0.0, 10.0, 35.0 + endingActiveComeback, 60.0 + approachingActiveComeback, 85.0 + endingActiveComeback, 110.0 + approachingActiveComeback};
  private static final double[] kComebackEndActive = {10.0, 35.0 + endingActiveComeback, 60.0 + approachingActiveComeback, 85.0 + endingActiveComeback, 110.0 + approachingActiveComeback, 140.0};
  private static final double[] kComebackStartInactive = {0.0, 10.0 + endingActiveComeback, 35.0 + approachingActiveComeback, 60.0 + endingActiveComeback, 85.0 + approachingActiveComeback, 110.0};
  private static final double[] kComebackEndInactive = {10.0 + endingActiveComeback, 35.0 + approachingActiveComeback, 60.0 + endingActiveComeback, 85.0 + approachingActiveComeback, 110.0, 140.0};

  private static Supplier<Optional<Boolean>> allianceWinOverride = () -> Optional.empty();

  public static Optional<Boolean> setAllianceWinOverride() {
      return allianceWinOverride.get();
  }

  public static Optional<Boolean> getAllianceWinOverride() {
    return allianceWinOverride.get();
  }

  public static Alliance getFirstActiveAlliance() {
    var alliance = WaltDriverStation.getAlliance().orElse(Alliance.Blue);

    // Return override value
    var winOverride = getAllianceWinOverride();
    if (!winOverride.isEmpty()) {
      return winOverride.get()
          ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
          : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
    }

    // Return FMS value
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    // Return default value
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  /** Starts the timer at the begining of teleop. */
  public static void initialize() {
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    boolean[] currentSchedule;
    Alliance startAlliance = getFirstActiveAlliance();
    currentSchedule =
        startAlliance == WaltDriverStation.getAlliance().orElse(Alliance.Blue)
            ? activeSchedule
            : inactiveSchedule;
    return currentSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
    double currentTime = shiftTimer.get();
    double stateTimeElapsed = shiftTimer.get();
    double stateTimeRemaining = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isEnabled()) {
      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        // After last shift, so assume endgame
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      // If the state is the same as the last shift, combine the elapsed time
      if (currentShiftIndex > 0) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
          stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
        }
      }

      // If the state is the same as the next shift, combine the remaining time
      if (currentShiftIndex < shiftEndTimes.length - 1) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
          stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
        }
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = shiftsEnums[currentShiftIndex];
    }
    ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
    return shiftInfo;
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  public static ShiftInfo getComebackShiftInfo() {
    boolean[] shiftSchedule = getSchedule();
    return shiftSchedule[1]
        ? getShiftInfo(shiftSchedule, kComebackStartActive, kComebackEndActive)
        : getShiftInfo(shiftSchedule, kComebackStartInactive, kComebackEndInactive);
  }

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] shiftSchedule = getSchedule();
    return shiftSchedule[1]
        ? getShiftInfo(shiftSchedule, kShiftedStartActive, kShiftedEndActive)
        : getShiftInfo(shiftSchedule, kShiftedStartInactive, kShiftedEndInactive);
  }
  /**
   * Determines whether the robot can prefire optimally in such a manner that you shoot before the hub turns active,
   * and the fuel will count as scored
   * <p> Meant to be used as a Rumble Trigger, so that the drivers will get rumble when optimal shooting.
   * @return true if optimal prefire time
   */
  public static BooleanSupplier optimalPrefireTime() {
   return () -> {
        boolean shiftedActive = getShiftedShiftInfo().active;
        boolean realActive = getOfficialShiftInfo().active;

        return (shiftedActive && !realActive) || (!shiftedActive && realActive);
    };
  }

  public static BooleanSupplier comebackTime() {
    return () -> {
      boolean realActive = getOfficialShiftInfo().active;
      boolean comebackActive = getComebackShiftInfo().active;

      return (comebackActive && !realActive) || (!comebackActive && realActive);
    };
  }
}
