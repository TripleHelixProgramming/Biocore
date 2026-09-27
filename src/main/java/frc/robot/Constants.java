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
import frc.lib.hardware.CANChain;
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
   * <p>Each bus declares its {@code CHAIN} after its {@code BUS_ID}, then its devices in
   * daisy-chain order: the first device is wired closest to the SystemCore port, and each next line
   * is the next device along the cable. Java runs these lines in the order written, so the order of
   * the lines is the chain order, and a device's position is its CAN index (see {@link CANChain}).
   *
   * <p>To trace a bus: start at its SystemCore port and follow the CAN wires to the terminator,
   * reordering the device lines to match. The bus is assumed to be one line with the SystemCore at
   * one end. A device on a side branch goes at the point where the branch leaves the main line.
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

      // Devices in daisy-chain order from the SystemCore
      public static final CANChain CHAIN = new CANChain(BUS_ID);
      public static final CANChain.Device PD = CHAIN.add(1, "Power distribution");
      public static final CANChain.Device GYRO = CHAIN.add(0, "Gyro");

      /** When and by whom the chain order was traced from the wiring, or null if not yet. */
      public static final String CHAIN_ORDER_TRACED = null;
    }

    /**
     * SystemCore CAN bus 1 — high-speed bus for swerve drivetrain devices. Supports TalonFX motor
     * controllers and CANcoders.
     */
    public static final class SC1 {
      public static final String NAME = "SC1";
      public static final CANPort BUS_ID = CANPort.CAN_S1;
      public static final CANBus BUS = new CANBus(BUS_ID);

      // Devices in daisy-chain order from the SystemCore
      public static final CANChain CHAIN = new CANChain(BUS_ID);
      public static final CANChain.Device BACK_LEFT_DRIVE = CHAIN.add(10, "BackLeft drive");
      public static final CANChain.Device BACK_RIGHT_DRIVE = CHAIN.add(18, "BackRight drive");
      public static final CANChain.Device FRONT_RIGHT_DRIVE = CHAIN.add(20, "FrontRight drive");
      public static final CANChain.Device FRONT_LEFT_DRIVE = CHAIN.add(28, "FrontLeft drive");
      public static final CANChain.Device BACK_LEFT_TURN = CHAIN.add(11, "BackLeft turn");
      public static final CANChain.Device BACK_RIGHT_TURN = CHAIN.add(19, "BackRight turn");
      public static final CANChain.Device FRONT_RIGHT_TURN = CHAIN.add(21, "FrontRight turn");
      public static final CANChain.Device FRONT_LEFT_TURN = CHAIN.add(29, "FrontLeft turn");
      public static final CANChain.Device BACK_RIGHT_TURN_ABS_ENC =
          CHAIN.add(31, "BackRight turn encoder");
      public static final CANChain.Device FRONT_RIGHT_TURN_ABS_ENC =
          CHAIN.add(33, "FrontRight turn encoder");
      public static final CANChain.Device FRONT_LEFT_TURN_ABS_ENC =
          CHAIN.add(43, "FrontLeft turn encoder");
      public static final CANChain.Device BACK_LEFT_TURN_ABS_ENC =
          CHAIN.add(45, "BackLeft turn encoder");

      /** When and by whom the chain order was traced from the wiring, or null if not yet. */
      public static final String CHAIN_ORDER_TRACED = null;
    }
  }
}
