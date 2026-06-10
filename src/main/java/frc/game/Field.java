package frc.game;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Field {
  // Measured
  public static final Distance centerField_x_pos = Inches.of(325.06);
  public static final Distance centerField_y_pos = Inches.of(158.32);

  // Constructed
  public static final Distance field_x_len = centerField_x_pos.times(2);
  public static final Distance field_y_len = centerField_y_pos.times(2);

  // private static final Pose2d fieldCenter =
  //     new Pose2d(new Translation2d(centerField_x_pos, centerField_y_pos), Rotation2d.kZero);

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
        new Pose2d(center.getX() - x, center.getY() - y, Rotation2d.kZero),
        new Pose2d(center.getX() - x, center.getY() + y, Rotation2d.kZero),
        new Pose2d(center.getX() + x, center.getY() + y, Rotation2d.kZero),
        new Pose2d(center.getX() + x, center.getY() - y, Rotation2d.kZero),
        new Pose2d(center.getX() - x, center.getY() - y, Rotation2d.kZero) // close loop
        );
  }
}
