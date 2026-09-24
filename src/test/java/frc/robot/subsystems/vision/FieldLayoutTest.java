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
