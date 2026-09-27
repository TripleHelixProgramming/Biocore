// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;

class CANChainTest {
  private static final List<CANChain.Device> CHAIN =
      List.of(
          new CANChain.Device(28, "FrontLeft drive"),
          new CANChain.Device(29, "FrontLeft turn"),
          new CANChain.Device(43, "FrontLeft turn encoder"),
          new CANChain.Device(20, "FrontRight drive"),
          new CANChain.Device(21, "FrontRight turn"));

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
            + " Check #2 FrontLeft turn encoder (ID 43)'s outgoing connector, the cable, and"
            + " #3 FrontRight drive (ID 20)'s incoming connector.",
        CANChain.hint("SC1", CHAIN, 3, "2026-10-03 A.Student"));
  }

  @Test
  void hintAtSystemCoreNamesThePort() {
    assertEquals(
        "CAN chain break on SC1 (order traced 2026-10-03 A.Student): no device responds."
            + " Check the SystemCore SC1 port and plug, and the cable to"
            + " #0 FrontLeft drive (ID 28).",
        CANChain.hint("SC1", CHAIN, 0, "2026-10-03 A.Student"));
  }

  @Test
  void matchingChainAndSourcesAreValid() {
    assertNull(CANChain.validate(CHAIN, Set.of(28, 29, 43, 20, 21)));
  }

  @Test
  void missingSourceIsReported() {
    assertEquals(
        "FrontRight turn (ID 21) has no connection source",
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
    var chain = List.of(new CANChain.Device(10, "A"), new CANChain.Device(10, "B"));
    assertEquals("CAN ID 10 appears twice in the chain", CANChain.validate(chain, Set.of(10)));
  }

  @Test
  void addReturnsTheId() {
    assertEquals(28, new CANChain(CANPort.CAN_S1).add(28, "FrontLeft drive"));
  }

  @Test
  void devicesKeepDeclarationOrder() {
    var chain = new CANChain(CANPort.CAN_S1);
    chain.add(28, "FrontLeft drive");
    chain.add(29, "FrontLeft turn");
    chain.add(10, "BackLeft drive");
    assertEquals(
        List.of(
            new CANChain.Device(28, "FrontLeft drive"),
            new CANChain.Device(29, "FrontLeft turn"),
            new CANChain.Device(10, "BackLeft drive")),
        chain.devices());
  }

  @Test
  void addAfterReadThrows() {
    var chain = new CANChain(CANPort.CAN_S1);
    chain.add(28, "FrontLeft drive");
    chain.devices();
    var error = assertThrows(IllegalStateException.class, () -> chain.add(29, "FrontLeft turn"));
    assertTrue(error.getMessage().contains("FrontLeft turn (ID 29)"));
  }

  @Test
  void devicesCannotBeModified() {
    var chain = new CANChain(CANPort.CAN_S1);
    chain.add(28, "FrontLeft drive");
    assertThrows(
        UnsupportedOperationException.class,
        () -> chain.devices().add(new CANChain.Device(1, "extra")));
  }

  @Test
  void connectionsOnKeepsOnlyThatBus() {
    BooleanSupplier up = () -> true;
    BooleanSupplier down = () -> false;
    var drive =
        Map.of(
            new CANChain.Address(CANPort.CAN_S1, 28), up,
            new CANChain.Address(CANPort.CAN_S0, 0), down);
    var pd = Map.of(new CANChain.Address(CANPort.CAN_S0, 1), up);

    var sc1 = CANChain.connectionsOn(CANPort.CAN_S1, drive, pd);
    assertEquals(Set.of(28), sc1.keySet());
    assertTrue(sc1.get(28).getAsBoolean());

    var sc0 = CANChain.connectionsOn(CANPort.CAN_S0, drive, pd);
    assertEquals(Set.of(0, 1), sc0.keySet());
    assertFalse(sc0.get(0).getAsBoolean());
  }

  @Test
  void sameIdOnDifferentBusesIsAllowed() {
    var a = Map.of(new CANChain.Address(CANPort.CAN_S0, 10), (BooleanSupplier) () -> true);
    var b = Map.of(new CANChain.Address(CANPort.CAN_S1, 10), (BooleanSupplier) () -> false);
    assertEquals(Set.of(10), CANChain.connectionsOn(CANPort.CAN_S1, a, b).keySet());
  }

  @Test
  void sameAddressFromTwoSourcesThrows() {
    var a = Map.of(new CANChain.Address(CANPort.CAN_S1, 10), (BooleanSupplier) () -> true);
    var b = Map.of(new CANChain.Address(CANPort.CAN_S1, 10), (BooleanSupplier) () -> false);
    var error =
        assertThrows(
            IllegalArgumentException.class, () -> CANChain.connectionsOn(CANPort.CAN_S1, a, b));
    assertEquals("CAN ID 10 on CAN_S1 is reported twice", error.getMessage());
  }
}
