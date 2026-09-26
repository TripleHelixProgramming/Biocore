// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

/**
 * Pure helpers for Phoenix 6 firmware status. They take plain strings and ints, not Phoenix types,
 * so they can be unit tested without loading Phoenix.
 */
public final class PhoenixFirmware {
  private PhoenixFirmware() {}

  /**
   * Returns true when a {@code setControl} status means Phoenix is blocking the motor's output.
   *
   * <p>Phoenix compares the device's firmware compliancy with the API's. On a mismatch it sends a
   * neutral output instead of the request and returns {@code FirmwareTooOld} or {@code ApiTooOld}
   * from {@code setControl} ({@code ParentDevice.setControlPrivate}, Phoenix 26.70.0-alpha-2).
   *
   * @param controlStatus the {@code StatusCode} name returned by {@code setControl}
   */
  public static boolean isBlocked(String controlStatus) {
    return "FirmwareTooOld".equals(controlStatus) || "ApiTooOld".equals(controlStatus);
  }

  /**
   * Formats a device's four-byte {@code getVersion()} value as "major.minor.bugfix.build".
   *
   * <p>The byte order, most significant byte first, is an assumption: Phoenix documents only a
   * "four byte value". Compare the result with {@code getVersionMajor()} and the other per-byte
   * signals before relying on it.
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
