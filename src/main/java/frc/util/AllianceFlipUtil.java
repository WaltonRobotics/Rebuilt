// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.util;

import static frc.robot.Constants.FieldK.kFieldLengthMeters;
import static frc.robot.Constants.FieldK.kFieldWidthMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
  /**
   * Flips the x-coordinate from blue alliance to red alliance.
   * Calculates the new x-coordinate when flipping from the blue side to the red side on a field with a given length.
   * @param xCoordinateMeters The x-coordinate in meters from the blue alliance's perspective.
   * @return The flipped x-coordinate in meters, relative to the red alliance's perspective.
   */
  public static double flipXCoordinate(double xCoordinateMeters) {
    return kFieldLengthMeters - xCoordinateMeters;
  }

  /**
   * Flips the y-coordinate from blue alliance to red alliance.
   * Calculates the new y-coordinate when flipping from the blue side to the red side on a field with a given width.
   * @param yCoordinateMeters The y-coordinate in meters from the blue alliance's perspective.
   * @return The flipped y-coordinate in meters, relative to the red alliance's perspective.
   */
  public static double flipYCoordinate(double yCoordinateMeters) {
    return kFieldWidthMeters - yCoordinateMeters;
  }

  /**
   * Flips an x-coordinate to the correct side of the field based on the current alliance color. 
   * @param xCoordinate The x-coordinate in meters from the blue alliance's perspective.
   * @return The flipped x-coordinate if the alliance is red, otherwise the x-coordinate.
   */
  public static double applyX(double xCoordinate) {
    if (shouldFlip()) {
      return flipXCoordinate(xCoordinate);
    } else {
      return xCoordinate;
    }
  }

  /**
   * Flips a y-coordinate to the correct side of the field based on the current alliance color.
   * @param yCoordinate The y-coordinate in meters from the blue alliance's perspective.
   * @return The flipped y-coordinate if the alliance is red, otherwise the y-coordinate.
   */
  public static double applyY(double yCoordinate) {
    if (shouldFlip()) {
      return flipYCoordinate(yCoordinate);
    } else {
      return yCoordinate;
    }
  }

  /**
   * Flips a translation by flipping its x and y coordinates.
   * @param translation The translation to be flipped.
   * @return new Translation2d with flipped coordinates
   */
  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(flipXCoordinate(translation.getX()), flipYCoordinate(translation.getY()));
  }

  /**
   * Flips a translation to the correct side of the field based on the current alliance color.
   * @param translation The translation to be applied.
   * @return The flipped translation if the alliance is red, otherwise the translation. 
   */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return flip(translation);
    } else {
      return translation;
    }
  }

  /**
   * Flips (Rotates) a rotation by 180 degrees.
   * @param rotation The rotation to be flipped.
   * @return new Rotation2d object rotated by 180 degrees
   */
  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }

  /**
   * Flips a rotation based on the current alliance color.
   * @param rotation The rotation to be applied.
   * @return The flipped rotation if the alliance is red, otherwise the rotation.
   */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return flip(rotation);
    } else {
      return rotation;
    }
  }

  /**
   * Flips a pose by flipping its translation and rotation.
   * @param pose The pose to be flipped.
   * @return new Pose2d with flippted translation and rotation.
   */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance color.
   * @param pose The pose to be applied.
   * @return The flipped pose if the allaince is red, otherwise the pose.
   */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return flip(pose);
    } else {
      return pose;
    }
  }

  /**
   * Flips a translation in 3D space by flipping the x and y coordinates, z-coordinate is unchanged.
   * @param translation3d The translation to be flipped.
   * @return new Translation3d object with the flipped x and y coordinates.
   */
  public static Translation3d flip(Translation3d translation3d) {
    return new Translation3d(
      flipXCoordinate(translation3d.getX()), flipYCoordinate(translation3d.getY()), translation3d.getZ());
  }

  /**
   * Flips a translation to the correct side of the field based on the current alliance color.
   * @param translation3d The translation to be applied
   * @return The flipped translation if the alliance is red, otherwise the translation.
   */
  public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlip()) {
      return flip(translation3d);
    } else {
      return translation3d;
    }
  }

  /**
   * Determine whether to flip or not, based on current alliance color.
   * @return true if the alliance is red, otherwise false.
   */
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
    // return true;
  }
}