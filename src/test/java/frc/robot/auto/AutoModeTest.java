// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/** Checks how the trajectories of an auto are joined into one path for plotting. */
class AutoModeTest {
  private static final int STEPS = 10;

  @Test
  void aLinearChainIsDrawnForwardOnce() {
    Pose2d[] a = line(0, 0, 1, 0);
    Pose2d[] b = line(1, 0, 2, 0);
    assertArrayEquals(concat(a, b), AutoMode.exploreBranches(List.of(a, b), 0));
  }

  @Test
  void siblingBranchesAreDrawnOutAndBackToTheirNode() {
    Pose2d[] root = line(0, 0, 1, 0);
    Pose2d[] up = line(1, 0, 2, 1);
    Pose2d[] down = line(1, 0, 2, -1);
    Pose2d[] joined = AutoMode.exploreBranches(List.of(root, up, down), 0);
    assertArrayEquals(concat(root, up, reversed(up), down), joined);
    assertContinuous(joined);
  }

  @Test
  void nestedBranchesReturnThroughEveryNode() {
    Pose2d[] root = line(0, 0, 1, 0);
    Pose2d[] inner = line(1, 0, 2, 0);
    Pose2d[] other = line(1, 0, 2, -1);
    Pose2d[] innerUp = line(2, 0, 3, 1);
    Pose2d[] innerDown = line(2, 0, 3, -1);
    Pose2d[] joined = AutoMode.exploreBranches(List.of(root, inner, other, innerUp, innerDown), 0);
    assertArrayEquals(
        concat(
            root,
            inner,
            innerUp,
            reversed(innerUp),
            innerDown,
            reversed(innerDown),
            reversed(inner),
            other),
        joined);
    assertContinuous(joined);
  }

  @Test
  void branchesThatRejoinDrawTheSharedPathForwardOnce() {
    Pose2d[] root = line(0, 0, 1, 0);
    Pose2d[] upper = line(1, 0, 2, 0);
    Pose2d[] lower = line(1, 0, 2, 0.001); // meets upper again within the node tolerance
    Pose2d[] shared = line(2, 0, 3, 0);
    Pose2d[] joined = AutoMode.exploreBranches(List.of(root, upper, lower, shared), 0);
    assertContinuous(joined);
    assertTrue(
        Collections.indexOfSubList(Arrays.asList(joined), Arrays.asList(shared)) >= 0,
        "The shared path is not drawn");
    assertEquals(
        Collections.indexOfSubList(Arrays.asList(joined), Arrays.asList(shared)),
        Collections.lastIndexOfSubList(Arrays.asList(joined), Arrays.asList(shared)),
        "The shared path is drawn forward more than once");
  }

  @Test
  void aDisconnectedPathIsStillDrawn() {
    Pose2d[] root = line(0, 0, 1, 0);
    Pose2d[] island = line(5, 5, 6, 5);
    assertArrayEquals(concat(root, island), AutoMode.exploreBranches(List.of(root, island), 0));
  }

  @Test
  void theRootIsDrawnFirstWhateverTheLoadOrder() {
    Pose2d[] a = line(0, 0, 1, 0);
    Pose2d[] b = line(1, 0, 2, 0);
    assertArrayEquals(concat(a, b), AutoMode.exploreBranches(List.of(b, a), 1));
  }

  /** No step between consecutive poses is longer than one step along a drawn path. */
  private static void assertContinuous(Pose2d[] joined) {
    double maxStep = Math.sqrt(2) / STEPS + 1e-9;
    for (int i = 1; i < joined.length; i++) {
      double step = joined[i - 1].getTranslation().getDistance(joined[i].getTranslation());
      assertTrue(step <= maxStep, "Jump of " + step + " m at pose " + i);
    }
  }

  private static Pose2d[] line(double x0, double y0, double x1, double y1) {
    Pose2d[] poses = new Pose2d[STEPS + 1];
    for (int i = 0; i <= STEPS; i++) {
      double t = (double) i / STEPS;
      poses[i] = new Pose2d(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, Rotation2d.ZERO);
    }
    return poses;
  }

  private static Pose2d[] reversed(Pose2d[] path) {
    Pose2d[] copy = path.clone();
    Collections.reverse(Arrays.asList(copy));
    return copy;
  }

  private static Pose2d[] concat(Pose2d[]... paths) {
    List<Pose2d> all = new ArrayList<>();
    for (Pose2d[] path : paths) all.addAll(Arrays.asList(path));
    return all.toArray(Pose2d[]::new);
  }
}
