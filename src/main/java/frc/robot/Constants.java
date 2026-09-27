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
import frc.lib.RobotMode;
import frc.lib.hardware.CANChainDevice;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final RobotMode simMode = RobotMode.SIM;
  public static final RobotMode currentMode = RobotBase.isReal() ? RobotMode.REAL : simMode;

  public static final class FeatureFlags {
    /** Enable to print loop timing when total exceeds 20ms. */
    public static final boolean PROFILING_ENABLED = false;

    public static final boolean LEDS_ENABLED = false;
    public static final boolean VISION_ENABLED = false;

    /** Enable to add the module forces from Choreo trajectories as drive feedforward. */
    public static final boolean TRAJECTORY_FORCE_FF = false;
  }

  public final class RobotConstants {
    public static final double NOMINAL_VOLTAGE = 12.0;
  }

  /**
   * Tolerances for pre-match feedback on how far the robot is from its auto start pose, shown on
   * the LEDs and the Driver Station display.
   */
  public static final class PoseSeekConstants {
    /** Heading tolerance in degrees. */
    public static final double HEADING_TOL_DEGREES = 3.0;

    /** Robot-relative forward/backward position tolerance in centimeters. */
    public static final double X_TOL_CM = 5.0;

    /** Robot-relative left/right position tolerance in centimeters. */
    public static final double Y_TOL_CM = 6.0;
  }

  public static final class USBStorageConstants {
    /** Free space on the USB log drive below which the robot warns the drive team. */
    public static final long LOW_FREE_BYTES = 2048L * 1024 * 1024;
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

  /**
   * The CAN buses and the devices on them.
   *
   * <p>Each bus lists its devices in a {@code Chain} enum, in daisy-chain order: the first constant
   * is the device wired closest to the SystemCore port, and each next constant is the next device
   * along the cable. A device's position in the enum is its CAN index, and the robot uses that
   * position for nothing else. The CAN ID and name of every device live only here, so device code
   * reads its ID from the enum, e.g. {@code SC1.Chain.FRONT_LEFT_DRIVE.id()}.
   *
   * <p>To trace a bus: start at its SystemCore port and follow the CAN wires to the terminator,
   * reordering the enum constants to match. The bus is assumed to be one line with the SystemCore
   * at one end. A device on a side branch goes at the point where the branch leaves the main line.
   * Then set {@code CHAIN_ORDER_TRACED} to the date and your name. While it is null, the robot
   * doesn't trust the order and gives no break-location hint for that bus.
   */
  public static final class CANBusPorts {

    /**
     * SystemCore CAN bus 0 — low-speed bus for support/sensing devices. Supports the power
     * distribution and gyro.
     */
    public static final class SC0 {
      public static final String NAME = "SC0";
      public static final CANPort BUS_ID = CANPort.CAN_S0;
      public static final CANBus BUS = new CANBus(BUS_ID);

      /** When and by whom the Chain order was traced from the wiring, or null if not yet. */
      public static final String CHAIN_ORDER_TRACED = null;

      /** SC0 devices in daisy-chain order from the SystemCore. */
      public enum Chain implements CANChainDevice {
        PD(1),
        GYRO(0);

        private final int id;

        Chain(int id) {
          this.id = id;
        }

        @Override
        public int id() {
          return id;
        }
      }
    }

    /**
     * SystemCore CAN bus 1 — high-speed bus for swerve drivetrain devices. Supports TalonFX motor
     * controllers and CANcoders.
     */
    public static final class SC1 {
      public static final String NAME = "SC1";
      public static final CANPort BUS_ID = CANPort.CAN_S1;
      public static final CANBus BUS = new CANBus(BUS_ID);

      /** When and by whom the Chain order was traced from the wiring, or null if not yet. */
      public static final String CHAIN_ORDER_TRACED = null;

      /** SC1 devices in daisy-chain order from the SystemCore. */
      public enum Chain implements CANChainDevice {
        BACK_LEFT_DRIVE(10),
        BACK_RIGHT_DRIVE(18),
        FRONT_RIGHT_DRIVE(20),
        FRONT_LEFT_DRIVE(28),
        BACK_LEFT_TURN(11),
        BACK_RIGHT_TURN(19),
        FRONT_RIGHT_TURN(21),
        FRONT_LEFT_TURN(29),
        BACK_RIGHT_TURN_ABS_ENC(31),
        FRONT_RIGHT_TURN_ABS_ENC(33),
        FRONT_LEFT_TURN_ABS_ENC(43),
        BACK_LEFT_TURN_ABS_ENC(45);

        private final int id;

        Chain(int id) {
          this.id = id;
        }

        @Override
        public int id() {
          return id;
        }
      }
    }
  }
}
