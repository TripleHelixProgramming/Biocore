package frc.game;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class GameState {

  public enum GamePhase {
    None("0:00 - 0:00"),
    Autonomous("0:20 - 0:00"),
    MidGame("2:20 - 0:30"),
    EndGame("0:30 - 0:00");

    public static final List<GamePhase> TELEOP = List.of(MidGame, EndGame);

    final double countDownFrom;
    final double countDownUntil;

    public double duration() {
      return countDownFrom - countDownUntil;
    }

    public double remainingAt(double atTime) {
      return atTime - countDownUntil;
    }

    private GamePhase(String timer) {
      var times = timer.split("-");
      this.countDownFrom = parseSeconds(times[0]);
      this.countDownUntil = parseSeconds(times[1]);
    }

    private static int parseSeconds(String time) {
      var parts = time.trim().split(":");
      return Integer.parseInt(parts[0]) * 60 + Integer.parseInt(parts[1]);
    }
  }

  private static Alliance myAlliance;

  public static GamePhase getCurrentPhase() {
    if (!DriverStation.isDSAttached() && !DriverStation.isFMSAttached()) {
      return GamePhase.None;
    }
    if (DriverStation.isAutonomous()) {
      return GamePhase.Autonomous;
    }
    // Must be in match and teleop
    var t = getMatchTime();
    for (var gamePhase : GamePhase.TELEOP) {
      if (t <= gamePhase.countDownFrom && t > gamePhase.countDownUntil) {
        return gamePhase;
      }
    }
    return GamePhase.None;
  }

  public static Alliance getMyAlliance() {
    if (myAlliance == null) {
      // myAlliance = DriverStation.getAlliance().orElse(null);
      myAlliance = Robot.allianceSelector.getAllianceColor();
    }
    return myAlliance;
  }

  public static double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  public static Optional<Alliance> getAlliance() {
    return DriverStation.getAlliance();
  }

  public static void logValues() {
    getMyAlliance();
    Logger.recordOutput("GameState/IsDSAttached", DriverStation.isDSAttached());
    Logger.recordOutput("GameState/IsFMSAttached", DriverStation.isFMSAttached());
    Logger.recordOutput("GameState/MatchType", DriverStation.getMatchType());
    Logger.recordOutput("GameState/IsAutonomus", DriverStation.isAutonomous());
    Logger.recordOutput("GameState/MatchTime", DriverStation.getMatchTime());
    Logger.recordOutput("GameState/Alliance", myAlliance);
    Logger.recordOutput("GameState/GameData", DriverStation.getGameSpecificMessage());
    Logger.recordOutput("GameState/CurrentPhase", getCurrentPhase());
  }
}
