package frc.robot.auto;

/** Minimal swerve trajectory sample, mirroring the fields Drive.followTrajectory() consumes. */
public record SwerveSample(
    double x, double y, double heading, double vx, double vy, double omega) {}
