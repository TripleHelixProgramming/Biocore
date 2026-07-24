// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

import org.wpilib.driverstation.Alliance;
import org.wpilib.util.Color;

public interface Util {
  /**
   * Map alliance object to corresponding color.
   *
   * @param alliance the alliance whose color we want
   * @return the appropriate color
   */
  public static Color allianceToColor(Alliance alliance) {
    return alliance == Alliance.BLUE ? Color.BLUE : Color.RED;
  }

  /**
   * Returns true if the numbners are within floating point precision
   *
   * @param a first value
   * @param b second value
   * @return true iff a and b are effectively equal
   */
  public static boolean nearlyEqual(double a, double b) {
    return Math.abs(a - b) < Math.ulp(1);
  }
}
