// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.auto;

import static frc.robot.auto.ChoreoProjectFiles.*;
import static org.junit.jupiter.api.Assertions.*;

import com.google.gson.JsonObject;
import org.junit.jupiter.api.Test;

/**
 * Checks the per-module forces in every trajectory against rigid-body dynamics. This pins down the
 * frame and module order of SwerveSample.moduleForcesX/Y, which the ChoreoLib javadoc gives as [FL,
 * FR, BL, BR]. Choreo's solver actually orders modules FL, BL, BR, FR (module_translations in
 * src-core/src/spec/project.rs) and expresses forces in the field frame.
 */
class ChoreoForceTest {
  private static final double FORCE_TOLERANCE_N = 1e-2;
  private static final double TORQUE_TOLERANCE_NM = 1e-2;

  @Test
  void moduleForcesObeyRigidBodyDynamics() {
    JsonObject config = project().getAsJsonObject("config");
    double mass = val(config.get("mass"));
    double inertia = val(config.get("inertia"));
    double frontX = val(config.getAsJsonObject("frontLeft").get("x"));
    double frontY = val(config.getAsJsonObject("frontLeft").get("y"));
    double backX = val(config.getAsJsonObject("backLeft").get("x"));
    double backY = val(config.getAsJsonObject("backLeft").get("y"));
    // Robot-frame module positions in Choreo's order: FL, BL, BR, FR
    double[][] modules = {{frontX, frontY}, {backX, backY}, {backX, -backY}, {frontX, -frontY}};

    trajectories()
        .forEach(
            (name, traj) -> {
              var samples = traj.getAsJsonObject("trajectory").getAsJsonArray("samples");
              assertFalse(samples.isEmpty(), name + ".traj has never been generated");
              for (var element : samples) {
                JsonObject s = element.getAsJsonObject();
                double t = s.get("t").getAsDouble();
                double heading = s.get("heading").getAsDouble();
                var fx = s.getAsJsonArray("fx");
                var fy = s.getAsJsonArray("fy");

                double sumFx = 0, sumFy = 0, torque = 0;
                for (int i = 0; i < 4; i++) {
                  double fxi = fx.get(i).getAsDouble();
                  double fyi = fy.get(i).getAsDouble();
                  // Rotate the module position into the field frame, where the forces live
                  double rx = modules[i][0] * Math.cos(heading) - modules[i][1] * Math.sin(heading);
                  double ry = modules[i][0] * Math.sin(heading) + modules[i][1] * Math.cos(heading);
                  sumFx += fxi;
                  sumFy += fyi;
                  torque += rx * fyi - ry * fxi;
                }

                String at = name + " at t=" + t;
                assertEquals(mass * s.get("ax").getAsDouble(), sumFx, FORCE_TOLERANCE_N, at);
                assertEquals(mass * s.get("ay").getAsDouble(), sumFy, FORCE_TOLERANCE_N, at);
                assertEquals(
                    inertia * s.get("alpha").getAsDouble(), torque, TORQUE_TOLERANCE_NM, at);
              }
            });
  }
}
