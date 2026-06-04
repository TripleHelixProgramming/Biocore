// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.leds;

/**
 * Constants for LED animations. Physical strip and segment configuration is defined in the {@link
 * LEDStrip} and {@link LEDSeries} enums.
 */
public final class LEDConstants {

  private LEDConstants() {}

  // ==================== ANIMATION CONFIGURATION ====================

  public static final int LEDS_PER_BLOCK = 1;
  public static final int LEDS_BETWEEN_BLOCKS = 1;

  // ==================== POSE SEEK TOLERANCES ====================

  /** Heading tolerance in degrees for pose-seek feedback. */
  public static final double POSE_SEEK_HEADING_TOL_DEGREES = 3.0;

  /** X position tolerance in centimeters for pose-seek feedback. */
  public static final double POSE_SEEK_X_TOL_CM = 5.0;

  /** Y position tolerance in centimeters for pose-seek feedback. */
  public static final double POSE_SEEK_Y_TOL_CM = 6.0;
}
