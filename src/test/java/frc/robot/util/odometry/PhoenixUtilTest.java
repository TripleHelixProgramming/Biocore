// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util.odometry;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.StatusCode;
import org.junit.jupiter.api.Test;

class PhoenixUtilTest {
  @Test
  void returnsOkAfterFirstSuccess() {
    int[] calls = {0};
    StatusCode status =
        PhoenixUtil.tryUntilOk(
            5,
            () -> {
              calls[0]++;
              return calls[0] < 3 ? StatusCode.ConfigFailed : StatusCode.OK;
            });
    assertEquals(StatusCode.OK, status);
    assertEquals(3, calls[0]);
  }

  @Test
  void returnsLastErrorWhenEveryAttemptFails() {
    int[] calls = {0};
    StatusCode status =
        PhoenixUtil.tryUntilOk(
            5,
            () -> {
              calls[0]++;
              return calls[0] < 5 ? StatusCode.ConfigFailed : StatusCode.TxFailed;
            });
    assertEquals(StatusCode.TxFailed, status);
    assertEquals(5, calls[0]);
  }

  @Test
  void firstErrorPicksTheFirstFailure() {
    assertEquals(
        StatusCode.TxFailed,
        PhoenixUtil.firstError(StatusCode.OK, StatusCode.TxFailed, StatusCode.ConfigFailed));
    assertEquals(StatusCode.OK, PhoenixUtil.firstError(StatusCode.OK, StatusCode.OK));
  }
}
