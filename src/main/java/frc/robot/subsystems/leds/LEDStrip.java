// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;

/**
 * Defines the physical LED strips on the robot. Each strip is connected to a PWM port and has a
 * fixed number of LEDs.
 *
 * <p>Modify this enum when the physical LED layout changes between seasons.
 */
public enum LEDStrip {
  MAIN(0, 36);
  // Add more strips here as needed, e.g.:
  // FRONT(8, 60),
  // BACK(7, 30);

  private final int port;
  private final int length;
  private AddressableLED led;
  private AddressableLEDBuffer buffer;
  LEDStrip(int port, int length) {
    this.port = port;
    this.length = length;
  }

  public int getLength() {
    return length;
  }

  /** Gets the LED buffer, creating it if necessary. */
  public AddressableLEDBuffer getBuffer() {
    if (buffer == null) {
      led = new AddressableLED(port);
      buffer = new AddressableLEDBuffer(length);
      led.setLength(length);
    }
    return buffer;
  }

  /** Creates a view of a portion of this strip's buffer. */
  public AddressableLEDBufferView createView(int start, int end) {
    return getBuffer().createView(start, end);
  }

  /** Creates a reversed view of a portion of this strip's buffer. */
  public AddressableLEDBufferView createReversedView(int start, int end) {
    return getBuffer().createView(start, end).reversed();
  }

  /** Pushes buffer data to the physical LED strip. */
  public void update() {
    if (led != null) {
      led.setData(buffer);
    }
  }

  /** Updates all physical LED strips with their current buffer data. */
  public static void updateAll() {
    for (LEDStrip strip : values()) {
      strip.update();
    }
  }
}
