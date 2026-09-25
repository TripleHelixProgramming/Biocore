// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.auto;

import static frc.robot.auto.ChoreoProjectFiles.*;
import static org.junit.jupiter.api.Assertions.*;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

/**
 * Guards against Choreo files that are out of date. Choreo's GUI and CLI only solve a trajectory
 * when asked, so an edit to a path, a variable, or the robot config can leave the deployed samples
 * behind the parameters they claim to follow.
 */
class ChoreoProjectTest {
  // An expression that is only a reference to a variable: "Name" or "Pose.x", "Pose.y",
  // "Pose.heading"
  private static final Pattern VARIABLE_REFERENCE =
      Pattern.compile("^([A-Za-z_]\\w*)(?:\\.(x|y|heading))?$");

  @Test
  void projectHasTrajectories() {
    assertFalse(trajectories().isEmpty(), "No .traj files in " + CHOREO_DIR.toAbsolutePath());
  }

  /** Choreo's own up_to_date check: the parameters were not edited after the last solve. */
  @Test
  void trajectoriesWereSolvedWithTheirCurrentParameters() {
    trajectories()
        .forEach(
            (name, traj) ->
                assertEquals(
                    traj.get("snapshot"),
                    values(traj.get("params")),
                    name + ".traj was edited after it was generated; regenerate it in Choreo"));
  }

  /** Choreo's own config_up_to_date check: the robot config was not edited after the solve. */
  @Test
  void trajectoriesWereSolvedWithTheCurrentRobotConfig() {
    JsonElement config = values(project().get("config"));
    trajectories()
        .forEach(
            (name, traj) ->
                assertEquals(
                    config,
                    traj.getAsJsonObject("trajectory").get("config"),
                    name + ".traj was generated with a different robot config; regenerate it"));
  }

  /**
   * Event markers are referenced by name strings in robot code, which Choreo code generation does
   * not cover. Until the first AutoFactory.bind() exists, no trajectory may carry an event.
   */
  @Test
  void trajectoriesHaveNoEventMarkers() {
    trajectories()
        .forEach(
            (name, traj) ->
                assertEquals(
                    0,
                    traj.getAsJsonArray("events").size(),
                    name + ".traj has event markers but nothing binds commands to them"));
  }

  /**
   * Only the Choreo GUI evaluates expressions; the CLI solves with the stored values. A waypoint
   * that references a variable must therefore store that variable's current value.
   */
  @Test
  void variableReferencesStoreTheVariableValue() {
    JsonObject variables = project().getAsJsonObject("variables");
    List<String> mismatches = new ArrayList<>();
    trajectories()
        .forEach((name, traj) -> checkReferences(name, traj.get("params"), variables, mismatches));
    assertTrue(mismatches.isEmpty(), String.join("\n", mismatches));
  }

  private static void checkReferences(
      String where, JsonElement element, JsonObject variables, List<String> mismatches) {
    if (isExpr(element)) {
      Matcher m = VARIABLE_REFERENCE.matcher(element.getAsJsonObject().get("exp").getAsString());
      if (!m.matches()) return;
      Double expected = variableValue(variables, m.group(1), m.group(2));
      if (expected != null && Math.abs(expected - val(element)) > 1e-9) {
        mismatches.add(where + ": " + m.group() + " stores " + val(element) + ", not " + expected);
      }
    } else if (element.isJsonObject()) {
      element
          .getAsJsonObject()
          .entrySet()
          .forEach(
              e -> checkReferences(where + "." + e.getKey(), e.getValue(), variables, mismatches));
    } else if (element.isJsonArray()) {
      var array = element.getAsJsonArray();
      for (int i = 0; i < array.size(); i++) {
        checkReferences(where + "[" + i + "]", array.get(i), variables, mismatches);
      }
    }
  }

  private static Double variableValue(JsonObject variables, String name, String component) {
    if (component == null) {
      JsonObject expressions = variables.getAsJsonObject("expressions");
      return expressions.has(name) ? val(expressions.getAsJsonObject(name).get("var")) : null;
    }
    JsonObject poses = variables.getAsJsonObject("poses");
    return poses.has(name) ? val(poses.getAsJsonObject(name).get(component)) : null;
  }
}
