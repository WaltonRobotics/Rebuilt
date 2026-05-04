// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.util;


import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.driverstation.DriverStation.Alliance;
import frc.robot.FieldConstants;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
  private static final Rotation3d kFlipRotation3d = new Rotation3d(0.0, 0.0, Math.PI);
  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return shouldFlip()
        ? new Translation2d(applyX(translation.getX()), applyY(translation.getY()))
        : translation;
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Translation3d apply(Translation3d translation) {
    return shouldFlip()
        ? new Translation3d(applyX(translation.getX()), applyY(translation.getY()), translation.getZ())
        : translation;
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(kFlipRotation3d) : rotation;
  }

  public static Pose3d apply(Pose3d pose) {
    return shouldFlip()
        ? new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  //kept this method, since I don't know what disableHAL is... until that point this shall stay :D
  public static boolean shouldFlip() {
    Optional<Alliance> alliance = WaltDriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
    // return true;
  }
}