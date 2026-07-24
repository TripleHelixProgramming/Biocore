// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.game;

import frc.robot.Robot;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;

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
    if (!RobotState.isDSAttached() && !RobotState.isFMSAttached()) {
      return GamePhase.None;
    }
    if (RobotState.isAutonomous()) {
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
      // myAlliance = MatchState.getAlliance().orElse(null);
      myAlliance = Robot.allianceSelector.getAllianceColor();
    }
    return myAlliance;
  }

  public static double getMatchTime() {
    return MatchState.getMatchTime();
  }

  public static Optional<Alliance> getAlliance() {
    return MatchState.getAlliance();
  }

  public static void logValues() {
    getMyAlliance();
    Logger.recordOutput("GameState/IsDSAttached", RobotState.isDSAttached());
    Logger.recordOutput("GameState/IsFMSAttached", RobotState.isFMSAttached());
    Logger.recordOutput("GameState/MatchType", MatchState.getMatchType());
    Logger.recordOutput("GameState/IsAutonomus", RobotState.isAutonomous());
    Logger.recordOutput("GameState/MatchTime", MatchState.getMatchTime());
    Logger.recordOutput("GameState/Alliance", myAlliance);
    Logger.recordOutput("GameState/GameData", MatchState.getGameData().orElse(""));
    Logger.recordOutput("GameState/CurrentPhase", getCurrentPhase());
  }
}
