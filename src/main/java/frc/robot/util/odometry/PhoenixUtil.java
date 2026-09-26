// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util.odometry;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtil {
  /**
   * Attempts to run the command until no error is produced.
   *
   * @return the status of the last attempt: OK on success, otherwise the final error
   */
  public static StatusCode tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    StatusCode status = StatusCode.OK;
    for (int i = 0; i < maxAttempts; i++) {
      status = command.get();
      if (status.isOK()) break;
    }
    return status;
  }

  /** Returns the first status that is not OK, or OK if every status is OK. */
  public static StatusCode firstError(StatusCode... statuses) {
    for (StatusCode status : statuses) {
      if (!status.isOK()) return status;
    }
    return StatusCode.OK;
  }
}
