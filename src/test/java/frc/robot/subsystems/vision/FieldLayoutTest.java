// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.wpilib.fields.Field;

class FieldLayoutTest {

  @Test
  void defaultLayoutLoadsWithTags() {
    Field layout = Vision.getAprilTagLayout();

    assertTrue(layout.hasTags(), "Default field has no AprilTag metadata");
    assertFalse(layout.getTags().isEmpty(), "Default field has an empty tag list");
    assertTrue(layout.getFieldLength() > 0.0);
    assertTrue(layout.getFieldWidth() > 0.0);

    int firstId = layout.getTags().get(0).getID();
    assertTrue(layout.getTagPose(firstId).isPresent(), "Tag " + firstId + " has no pose");
  }
}
