// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified work Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class FeatureFlags {
    /** Enable to print loop timing when total exceeds 20ms. */
    public static final boolean PROFILING_ENABLED = false;
  }

  public final class RobotConstants {
    public static final double NOMINAL_VOLTAGE = 12.0;
  }

  public static final class MotorConstants {
    public static final class NEOConstants {
      public static final int DEFAULT_SUPPLY_CURRENT_LIMIT = 40;
      public static final int DEFAULT_STATOR_CURRENT_LIMIT = 60;
    }

    public static final class NEO550Constants {
      public static final int DEFAULT_SUPPLY_CURRENT_LIMIT = 5;
      public static final int DEFAULT_STATOR_CURRENT_LIMIT = 10;
    }

    public static final class NEOVortexConstants {
      public static final int DEFAULT_SUPPLY_CURRENT_LIMIT = 60;
      public static final int DEFAULT_STATOR_CURRENT_LIMIT = 100;
    }

    public static final class KrakenX60Constants {
      public static final int DEFAULT_SUPPLY_CURRENT_LIMIT = 60;
      public static final int DEFAULT_STATOR_CURRENT_LIMIT = 100;
    }
  }

  public static final class DIOPorts {
    // max length is 8
    public static final int[] AUTONOMOUS_MODE_SELECTOR = {0, 1, 2};

    public static final int ALLIANCE_COLOR_SELECTOR = 3;
  }

  public static final class CANBusPorts {

    public static final class CAN2 {
      public static final CANBus BUS = CANBus.roboRIO();

      // Power distribution
      public static final int PD = 1;

      // Drivetrain
      public static final int GYRO = 0;
    }

    public static final class CANHD {
      // CAN bus that the devices are located on;
      // All swerve devices must share the same CAN bus
      public static final CANBus BUS = new CANBus("canivore");

      // Drivetrain
      public static final int BACK_LEFT_DRIVE = 10;
      public static final int BACK_RIGHT_DRIVE = 18;
      public static final int FRONT_RIGHT_DRIVE = 20;
      public static final int FRONT_LEFT_DRIVE = 28;

      public static final int BACK_LEFT_TURN = 11;
      public static final int BACK_RIGHT_TURN = 19;
      public static final int FRONT_RIGHT_TURN = 21;
      public static final int FRONT_LEFT_TURN = 29;

      public static final int BACK_RIGHT_TURN_ABS_ENC = 31;
      public static final int FRONT_RIGHT_TURN_ABS_ENC = 33;
      public static final int FRONT_LEFT_TURN_ABS_ENC = 43;
      public static final int BACK_LEFT_TURN_ABS_ENC = 45;
    }
  }
}
