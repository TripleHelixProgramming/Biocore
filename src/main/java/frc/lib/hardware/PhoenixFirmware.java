// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

/**
 * Formats Phoenix 6 firmware versions. It takes a plain int, not a Phoenix type, so it can be unit
 * tested without loading Phoenix.
 */
public final class PhoenixFirmware {
  private PhoenixFirmware() {}

  /**
   * Formats a device's four-byte {@code getVersion()} value as "major.minor.bugfix.build".
   *
   * <p>The most significant byte is the major version. Phoenix documents only a "four byte value";
   * this order matched {@code getVersionMajor()}, {@code getVersionMinor()}, {@code
   * getVersionBugfix()} and {@code getVersionBuild()} on Phoenix sim devices ({@code 0x1A460000} is
   * 26.70.0.0).
   *
   * @param version the raw version value
   * @return the formatted version, or "" for 0, which means the version is unknown
   */
  public static String format(int version) {
    if (version == 0) return "";
    return ((version >>> 24) & 0xFF)
        + "."
        + ((version >>> 16) & 0xFF)
        + "."
        + ((version >>> 8) & 0xFF)
        + "."
        + (version & 0xFF);
  }
}
