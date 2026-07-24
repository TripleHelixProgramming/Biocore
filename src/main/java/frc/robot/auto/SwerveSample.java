// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.auto;

/** Minimal swerve trajectory sample, mirroring the fields Drive.followTrajectory() consumes. */
public record SwerveSample(
    double x, double y, double heading, double vx, double vy, double omega) {}
