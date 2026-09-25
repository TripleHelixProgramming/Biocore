// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.game;

import frc.robot.generated.ChoreoVars;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.shape.Rectangle2d;
import org.wpilib.units.measure.Distance;

/**
 * Field geometry. Every value comes from the variables in the Choreo project ({@code
 * src/main/deploy/choreo/Rho.chor}) through the generated {@link ChoreoVars}, so that robot code
 * and trajectories share one definition. Edit geometry in the Choreo GUI, not here.
 */
public class Field {
  public static final Distance field_x_len = ChoreoVars.FieldLength;
  public static final Distance field_y_len = ChoreoVars.FieldWidth;
  public static final Distance centerField_x_pos = ChoreoVars.FieldCenterX;
  public static final Distance centerField_y_pos = ChoreoVars.FieldCenterY;

  // private static final Pose2d fieldCenter =
  //     new Pose2d(new Translation2d(centerField_x_pos, centerField_y_pos), Rotation2d.ZERO);

  enum Region {
    Field(new Rectangle2d(new Translation2d(0, 0), new Translation2d(field_x_len, field_y_len)));

    private final Rectangle2d rect;

    private Region(Rectangle2d rect) {
      this.rect = rect;
    }

    public boolean contains(Pose2d pose) {
      return rect.contains(pose.getTranslation());
    }

    public Pose2d getCenter() {
      return rect.getCenter();
    }
  }

  public static void plotRegions() {
    for (Region region : Region.values()) {
      Logger.recordOutput(
          "Field/Regions/" + region.name(), rectangleToPoses(region.rect).toArray(new Pose2d[0]));
    }
  }

  private static List<Pose2d> rectangleToPoses(Rectangle2d rect) {
    Translation2d center = rect.getCenter().getTranslation();
    double x = rect.getXWidth() / 2.0;
    double y = rect.getYWidth() / 2.0;

    return List.of(
        new Pose2d(center.getX() - x, center.getY() - y, Rotation2d.ZERO),
        new Pose2d(center.getX() - x, center.getY() + y, Rotation2d.ZERO),
        new Pose2d(center.getX() + x, center.getY() + y, Rotation2d.ZERO),
        new Pose2d(center.getX() + x, center.getY() - y, Rotation2d.ZERO),
        new Pose2d(center.getX() - x, center.getY() - y, Rotation2d.ZERO) // close loop
        );
  }
}
