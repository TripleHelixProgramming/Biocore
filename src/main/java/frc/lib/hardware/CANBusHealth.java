// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

/**
 * Decides whether a CAN bus needs an alert, from one cycle of logged bus status at a time.
 *
 * <p>The states follow the CAN standard's fault confinement: a controller enters ErrorWarning when
 * an error counter reaches 96, ErrorPassive at 128, and BusOff when the transmit counter passes
 * 255. A few error frames on a healthy bus raise the counters without reaching those states, so the
 * raw counters are logged but do not trigger alerts.
 *
 * <p>This class uses only strings, numbers and the caller's timestamps, so it gives the same result
 * in log replay and can be unit tested without Phoenix or the HAL.
 */
public class CANBusHealth {
  /** How urgent a bus problem is. */
  public enum Severity {
    NONE,
    MEDIUM,
    HIGH
  }

  /** How long an alert stays active after the condition was last seen, in seconds. */
  public static final double HOLD_SECONDS = 0.5;

  /** How long the sample count may stay unchanged before the reader counts as stalled. */
  public static final double STALE_SECONDS = 1.5;

  private boolean hasPrevious = false;
  private long previousSampleCount;
  private long previousBusOffCount;
  private long previousRestartCount;
  private double lastSampleChangeTime;
  private double lastHighTime = Double.NEGATIVE_INFINITY;
  private double lastMediumTime = Double.NEGATIVE_INFINITY;

  /**
   * Returns the severity of one bus status sample.
   *
   * @param status the Phoenix StatusCode name of the status read ("OK" when the read worked)
   * @param state the Phoenix CANState name
   * @param busOffRose whether the bus-off count rose since the previous sample
   * @param restartRose whether the controller restart count rose since the previous sample
   * @param stale whether the reader has stopped producing samples
   */
  public static Severity severity(
      String status, String state, boolean busOffRose, boolean restartRose, boolean stale) {
    if (!"OK".equals(status)
        || "ErrorPassive".equals(state)
        || "BusOff".equals(state)
        || "Stopped".equals(state)
        || busOffRose
        || restartRose
        || stale) {
      return Severity.HIGH;
    }
    if ("ErrorWarning".equals(state)) return Severity.MEDIUM;
    return Severity.NONE;
  }

  /**
   * Takes one cycle of logged bus status.
   *
   * <p>A sample count of 0 means no status has been read, as in simulation or when replaying a log
   * recorded without a reader. Such cycles never count as stale. Counter increases are measured
   * from the first real sample, so counts left from before a code restart do not trigger an alert.
   *
   * @param now the current timestamp in seconds
   * @param sampleCount how many status reads the reader has completed
   * @param status the Phoenix StatusCode name of the status read
   * @param state the Phoenix CANState name
   * @param busOffCount the cumulative bus-off count
   * @param restartCount the cumulative controller restart count
   */
  public void update(
      double now,
      long sampleCount,
      String status,
      String state,
      long busOffCount,
      long restartCount) {
    boolean busOffRose = false;
    boolean restartRose = false;
    boolean stale = false;
    if (sampleCount > 0) {
      if (!hasPrevious) {
        hasPrevious = true;
        previousSampleCount = sampleCount;
        lastSampleChangeTime = now;
      } else {
        busOffRose = busOffCount > previousBusOffCount;
        restartRose = restartCount > previousRestartCount;
        if (sampleCount != previousSampleCount) {
          previousSampleCount = sampleCount;
          lastSampleChangeTime = now;
        }
        stale = now - lastSampleChangeTime > STALE_SECONDS;
      }
      previousBusOffCount = busOffCount;
      previousRestartCount = restartCount;
    }

    switch (severity(status, state, busOffRose, restartRose, stale)) {
      case HIGH -> lastHighTime = now;
      case MEDIUM -> lastMediumTime = now;
      case NONE -> {}
    }
  }

  /** Returns true while a high-severity problem was seen within the last {@link #HOLD_SECONDS}. */
  public boolean isHighActive(double now) {
    return now - lastHighTime < HOLD_SECONDS;
  }

  /** Returns true while a warning was seen within the last {@link #HOLD_SECONDS}. */
  public boolean isMediumActive(double now) {
    return now - lastMediumTime < HOLD_SECONDS;
  }
}
