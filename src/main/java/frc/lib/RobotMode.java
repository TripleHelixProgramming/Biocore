// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

/** The runtime mode used by AdvantageKit. */
public enum RobotMode {
  /** Running on a real robot. */
  REAL,

  /** Running a physics simulator. */
  SIM,

  /** Replaying from a log file. */
  REPLAY
}
