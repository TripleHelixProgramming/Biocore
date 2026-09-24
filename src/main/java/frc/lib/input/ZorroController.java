// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.input;

import java.util.Objects;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.HIDDevice;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;

/** Handle input from a RadioMaster Zorro controller connected to the Driver Station. */
public class ZorroController implements HIDDevice, TelemetryLoggable {
  private final GenericHID m_hid;

  // RadioMaster Zorro joystick axis
  public enum Axis {
    kLeftXAxis(0),
    kLeftYAxis(1),
    kLeftDial(2),
    kRightDial(3),
    kRightXAxis(4),
    kRightYAxis(5);

    public final int value;

    Axis(int value) {
      this.value = value;
    }

    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }

  // RadioMaster Zorro buttons
  public enum Button {
    kBDown(1),
    kBMid(2),
    kBUp(3),
    kEDown(4),
    kEUp(5),
    kAIn(6),
    kGIn(7),
    kCDown(8),
    kCMid(9),
    kCUp(10),
    kFDown(11),
    kFUp(12),
    kDIn(13),
    kHIn(14);

    public final int value;

    Button(int value) {
      this.value = value;
    }

    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

  /**
   * Construct an instance of a Zorro controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into (0-5).
   */
  public ZorroController(int port) {
    this(DriverStation.getGenericHID(port));
  }

  /**
   * Construct an instance of a Zorro controller with a GenericHID object.
   *
   * @param hid The GenericHID object to use for this controller.
   */
  public ZorroController(GenericHID hid) {
    m_hid = Objects.requireNonNull(hid, "Provided HID object cannot be null");
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public GenericHID getHID() {
    return m_hid;
  }

  public int getPort() {
    return m_hid.getPort();
  }

  public boolean isConnected() {
    return m_hid.isConnected();
  }

  public double getLeftXAxis() {
    return m_hid.getRawAxis(Axis.kLeftXAxis.value);
  }

  public double getLeftYAxis() {
    return m_hid.getRawAxis(Axis.kLeftYAxis.value);
  }

  public double getRightXAxis() {
    return m_hid.getRawAxis(Axis.kRightXAxis.value);
  }

  public double getRightYAxis() {
    return m_hid.getRawAxis(Axis.kRightYAxis.value);
  }

  public double getLeftDial() {
    return m_hid.getRawAxis(Axis.kLeftDial.value);
  }

  public double getRightDial() {
    return m_hid.getRawAxis(Axis.kRightDial.value);
  }

  public boolean getBDown() {
    return m_hid.getRawButton(Button.kBDown.value);
  }

  public boolean getBMid() {
    return m_hid.getRawButton(Button.kBMid.value);
  }

  public boolean getBUp() {
    return m_hid.getRawButton(Button.kBUp.value);
  }

  public boolean getEDown() {
    return m_hid.getRawButton(Button.kEDown.value);
  }

  public boolean getEUp() {
    return m_hid.getRawButton(Button.kEUp.value);
  }

  public boolean getAIn() {
    return m_hid.getRawButton(Button.kAIn.value);
  }

  public boolean getGIn() {
    return m_hid.getRawButton(Button.kGIn.value);
  }

  public boolean getCDown() {
    return m_hid.getRawButton(Button.kCDown.value);
  }

  public boolean getCMid() {
    return m_hid.getRawButton(Button.kCMid.value);
  }

  public boolean getCUp() {
    return m_hid.getRawButton(Button.kCUp.value);
  }

  public boolean getFDown() {
    return m_hid.getRawButton(Button.kFDown.value);
  }

  public boolean getFUp() {
    return m_hid.getRawButton(Button.kFUp.value);
  }

  public boolean getDIn() {
    return m_hid.getRawButton(Button.kDIn.value);
  }

  public boolean getHIn() {
    return m_hid.getRawButton(Button.kHIn.value);
  }

  @Override
  public void logTo(TelemetryTable table) {
    table.log("LeftXAxis", getLeftXAxis());
    table.log("LeftYAxis", getLeftYAxis());
    table.log("LeftDial", getLeftDial());
    table.log("RightDial", getRightDial());
    table.log("RightXAxis", getRightXAxis());
    table.log("RightYAxis", getRightYAxis());

    table.log("AIn", getAIn());

    table.log("BDown", getBDown());
    table.log("BMid", getBMid());
    table.log("BUp", getBUp());

    table.log("CDown", getCDown());
    table.log("CMid", getCMid());
    table.log("CUp", getCUp());

    table.log("DIn", getDIn());

    table.log("EDown", getEDown());
    table.log("EUp", getEUp());

    table.log("FDown", getFDown());
    table.log("FUp", getFUp());

    table.log("GIn", getGIn());

    table.log("HIn", getHIn());
  }

  @Override
  public String getTelemetryType() {
    return "HID:Zorro";
  }
}
