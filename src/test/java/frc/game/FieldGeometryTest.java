// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.game;

import static frc.robot.auto.ChoreoProjectFiles.*;
import static org.junit.jupiter.api.Assertions.*;

import com.google.gson.JsonObject;
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.vision.Vision;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Set;
import java.util.TreeSet;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.Measure;

/**
 * Field geometry has one source: the variables in the Choreo project. These tests check that the
 * generated ChoreoVars still matches the project, and that the project matches the real field.
 */
class FieldGeometryTest {
  // Choreo rounds generated values to 1e-7
  private static final double GENERATED_TOLERANCE = 1e-6;
  // The AprilTag layout gives field dimensions to the millimeter
  private static final double FIELD_SIZE_TOLERANCE_M = 1e-3;

  private final JsonObject variables = project().getAsJsonObject("variables");

  @Test
  void choreoVarsMatchesTheProject() throws IllegalAccessException {
    JsonObject expressions = variables.getAsJsonObject("expressions");
    Set<String> generated = new TreeSet<>();
    for (Field f : ChoreoVars.class.getFields()) {
      if (!Modifier.isStatic(f.getModifiers())) continue;
      generated.add(f.getName());
      assertTrue(expressions.has(f.getName()), f.getName() + " is no longer in the project");
      double expected = val(expressions.getAsJsonObject(f.getName()).get("var"));
      Object value = f.get(null);
      double actual =
          value instanceof Measure<?> m ? m.baseUnitMagnitude() : ((Number) value).doubleValue();
      assertEquals(expected, actual, GENERATED_TOLERANCE, f.getName() + " is stale");
    }
    assertEquals(expressions.keySet(), generated, "ChoreoVars is missing project variables");

    JsonObject poses = variables.getAsJsonObject("poses");
    Set<String> generatedPoses = new TreeSet<>();
    for (Field f : ChoreoVars.Poses.class.getFields()) {
      generatedPoses.add(f.getName());
      assertTrue(poses.has(f.getName()), f.getName() + " is no longer in the project");
      JsonObject expected = poses.getAsJsonObject(f.getName());
      Pose2d actual = (Pose2d) f.get(null);
      assertEquals(val(expected.get("x")), actual.getX(), GENERATED_TOLERANCE, f.getName());
      assertEquals(val(expected.get("y")), actual.getY(), GENERATED_TOLERANCE, f.getName());
      // Compare as angles: Rotation2d wraps +pi to -pi
      Rotation2d expectedHeading = Rotation2d.fromRadians(val(expected.get("heading")));
      assertEquals(
          0.0,
          actual.getRotation().minus(expectedHeading).getRadians(),
          GENERATED_TOLERANCE,
          f.getName());
    }
    assertEquals(poses.keySet(), generatedPoses, "ChoreoVars.Poses is missing project poses");
  }

  /**
   * Only the Choreo GUI evaluates expressions. This recomputes the derived variables so that a
   * value edited outside the GUI cannot drift from its formula.
   */
  @Test
  void derivedVariablesFollowTheirFormulas() {
    JsonObject expressions = variables.getAsJsonObject("expressions");
    double length = val(expressions.getAsJsonObject("FieldLength").get("var"));
    double width = val(expressions.getAsJsonObject("FieldWidth").get("var"));
    assertEquals(length / 2, val(expressions.getAsJsonObject("FieldCenterX").get("var")), 1e-9);
    assertEquals(width / 2, val(expressions.getAsJsonObject("FieldCenterY").get("var")), 1e-9);
  }

  @Test
  void fieldSizeMatchesTheAprilTagLayout() {
    var layout = Vision.getAprilTagLayout();
    assertEquals(
        layout.getFieldLength(),
        ChoreoVars.FieldLength.baseUnitMagnitude(),
        FIELD_SIZE_TOLERANCE_M);
    assertEquals(
        layout.getFieldWidth(), ChoreoVars.FieldWidth.baseUnitMagnitude(), FIELD_SIZE_TOLERANCE_M);
  }
}
