// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

class CANChainTest {
  private record Device(int id, String name) implements CANChainDevice {}

  private static final List<Device> CHAIN =
      List.of(
          new Device(28, "FRONT_LEFT_DRIVE"),
          new Device(29, "FRONT_LEFT_TURN"),
          new Device(43, "FRONT_LEFT_TURN_ABS_ENC"),
          new Device(20, "FRONT_RIGHT_DRIVE"),
          new Device(21, "FRONT_RIGHT_TURN"));

  private static boolean[] up(boolean... connected) {
    return connected;
  }

  @Test
  void allUpIsNoBreak() {
    assertEquals(-1, CANChain.findBreak(up(true, true, true, true, true)));
  }

  @Test
  void allDownIsBreakAtSystemCore() {
    assertEquals(0, CANChain.findBreak(up(false, false, false, false, false)));
  }

  @Test
  void cleanSplitsGiveFirstDownIndex() {
    assertEquals(1, CANChain.findBreak(up(true, false, false, false, false)));
    assertEquals(2, CANChain.findBreak(up(true, true, false, false, false)));
    assertEquals(3, CANChain.findBreak(up(true, true, true, false, false)));
  }

  @Test
  void onlyLastDeviceDownIsNoBreak() {
    assertEquals(-1, CANChain.findBreak(up(true, true, true, true, false)));
  }

  @Test
  void singleDeviceChainIsNeverABreak() {
    assertEquals(-1, CANChain.findBreak(up(false)));
    assertEquals(-1, CANChain.findBreak(up(true)));
  }

  @Test
  void emptyChainIsNoBreak() {
    assertEquals(-1, CANChain.findBreak(up()));
  }

  @Test
  void singleMidDeviceDownIsNoBreak() {
    assertEquals(-1, CANChain.findBreak(up(true, true, false, true, true)));
  }

  @Test
  void downThenUpIsNoBreak() {
    assertEquals(-1, CANChain.findBreak(up(true, false, false, true, false)));
    assertEquals(-1, CANChain.findBreak(up(false, false, true, true, true)));
  }

  @Test
  void hintNamesBothSidesOfTheBreak() {
    assertEquals(
        "CAN chain break on SC1 (order traced 2026-10-03 A.Student): #0–#2 respond, #3–#4 don't."
            + " Check #2 FRONT_LEFT_TURN_ABS_ENC (ID 43)'s outgoing connector, the cable, and"
            + " #3 FRONT_RIGHT_DRIVE (ID 20)'s incoming connector.",
        CANChain.hint("SC1", CHAIN, 3, "2026-10-03 A.Student"));
  }

  @Test
  void hintAtSystemCoreNamesThePort() {
    assertEquals(
        "CAN chain break on SC1 (order traced 2026-10-03 A.Student): no device responds."
            + " Check the SystemCore SC1 port and plug, and the cable to"
            + " #0 FRONT_LEFT_DRIVE (ID 28).",
        CANChain.hint("SC1", CHAIN, 0, "2026-10-03 A.Student"));
  }

  @Test
  void matchingChainAndSourcesAreValid() {
    assertNull(CANChain.validate(CHAIN, Set.of(28, 29, 43, 20, 21)));
  }

  @Test
  void missingSourceIsReported() {
    assertEquals(
        "FRONT_RIGHT_TURN (ID 21) has no connection source",
        CANChain.validate(CHAIN, Set.of(28, 29, 43, 20)));
  }

  @Test
  void unknownSourceIsReported() {
    assertEquals(
        "CAN ID 99 has a connection source but is not in the chain",
        CANChain.validate(CHAIN, Set.of(28, 29, 43, 20, 21, 99)));
  }

  @Test
  void duplicateChainIdIsReported() {
    var chain = List.of(new Device(10, "A"), new Device(10, "B"));
    assertEquals("CAN ID 10 appears twice in the chain", CANChain.validate(chain, Set.of(10)));
  }
}
