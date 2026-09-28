// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import static org.junit.jupiter.api.Assertions.*;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.wpilib.hardware.bus.CANPort;

class CANChainTest {
  private static CANChain sc1Chain() {
    var chain = new CANChain("SC1", CANPort.CAN_S1);
    chain.add(28, "FrontLeft drive");
    chain.add(29, "FrontLeft turn");
    chain.add(43, "FrontLeft turn encoder");
    chain.add(20, "FrontRight drive");
    chain.add(21, "FrontRight turn");
    return chain;
  }

  private static final List<CANChain.Device> CHAIN = sc1Chain().devices();

  /** Parses a pattern such as "11000" (1 = connected) into connection states. */
  private static boolean[] pattern(String upDown) {
    boolean[] connected = new boolean[upDown.length()];
    for (int i = 0; i < connected.length; i++) connected[i] = upDown.charAt(i) == '1';
    return connected;
  }

  @ParameterizedTest(name = "{0} -> {1}")
  @CsvSource({
    "11111, -1", // all up
    "00000, 0", // all down: break at the SystemCore
    "10000, 1",
    "11000, 2",
    "11100, 3",
    "11110, -1", // only the last device down: that device's own alert names it
    "0, -1", // one-device chain can't show a break
    "1, -1",
    "'', -1", // empty chain
    "11011, -1", // one device down mid-chain
    "10010, -1", // down, then up again
    "00111, -1",
  })
  void findBreakNeedsACleanSplitWithAtLeastTwoDown(String upDown, int expected) {
    assertEquals(expected, CANChain.findBreak(pattern(upDown)));
  }

  @Test
  void hintNamesBothSidesOfTheBreak() {
    assertEquals(
        "CAN chain break on SC1 (order traced 2026-10-03 A.Student): #0–#2 respond, #3–#4 don't."
            + " Check #2 FrontLeft turn encoder (ID 43)'s outgoing connector, the cable, and"
            + " #3 FrontRight drive (ID 20)'s incoming connector.",
        sc1Chain().hint(3, "2026-10-03 A.Student"));
  }

  @Test
  void hintAtSystemCoreNamesThePort() {
    assertEquals(
        "CAN chain break on SC1 (order traced 2026-10-03 A.Student): no device responds."
            + " Check the SystemCore SC1 port and plug, and the cable to"
            + " #0 FrontLeft drive (ID 28).",
        sc1Chain().hint(0, "2026-10-03 A.Student"));
  }

  @Test
  void chainWithEverySourceIsValid() {
    var sources = new HashSet<>(CHAIN);
    sources.add(new CANChain.Device(CANPort.CAN_S0, 1, "Power distribution")); // other bus
    assertNull(CANChain.validate(CHAIN, sources));
  }

  @Test
  void missingSourceIsReported() {
    var sources = new HashSet<>(CHAIN);
    sources.remove(CHAIN.get(4));
    assertEquals(
        "FrontRight turn (ID 21) has no connection source", CANChain.validate(CHAIN, sources));
  }

  @Test
  void duplicateChainIdIsReported() {
    var a = new CANChain.Device(CANPort.CAN_S1, 10, "A");
    var b = new CANChain.Device(CANPort.CAN_S1, 10, "B");
    assertEquals(
        "CAN ID 10 appears twice in the chain", CANChain.validate(List.of(a, b), Set.of(a, b)));
  }

  @Test
  void addReturnsTheDeviceOnTheChainsPort() {
    assertEquals(
        new CANChain.Device(CANPort.CAN_S1, 28, "FrontLeft drive"),
        new CANChain("SC1", CANPort.CAN_S1).add(28, "FrontLeft drive"));
  }

  @Test
  void devicesKeepDeclarationOrder() {
    var chain = new CANChain("SC1", CANPort.CAN_S1);
    chain.add(28, "FrontLeft drive");
    chain.add(29, "FrontLeft turn");
    chain.add(10, "BackLeft drive");
    assertEquals(
        List.of(
            new CANChain.Device(CANPort.CAN_S1, 28, "FrontLeft drive"),
            new CANChain.Device(CANPort.CAN_S1, 29, "FrontLeft turn"),
            new CANChain.Device(CANPort.CAN_S1, 10, "BackLeft drive")),
        chain.devices());
  }

  @Test
  void devicesCannotBeModified() {
    var chain = new CANChain("SC1", CANPort.CAN_S1);
    chain.add(28, "FrontLeft drive");
    assertThrows(
        UnsupportedOperationException.class,
        () -> chain.devices().add(new CANChain.Device(CANPort.CAN_S1, 1, "extra")));
  }
}
