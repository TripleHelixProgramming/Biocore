// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import org.junit.jupiter.api.Test;

class CANChainBuilderTest {
  @Test
  void addReturnsTheId() {
    var builder = new CANChainBuilder();
    assertEquals(28, builder.add(28, "FrontLeft drive"));
  }

  @Test
  void chainKeepsDeclarationOrder() {
    var builder = new CANChainBuilder();
    builder.add(28, "FrontLeft drive");
    builder.add(29, "FrontLeft turn");
    builder.add(10, "BackLeft drive");
    assertEquals(
        List.of(
            new CANChainDevice(28, "FrontLeft drive"),
            new CANChainDevice(29, "FrontLeft turn"),
            new CANChainDevice(10, "BackLeft drive")),
        builder.build());
  }

  @Test
  void addAfterBuildThrows() {
    var builder = new CANChainBuilder();
    builder.add(28, "FrontLeft drive");
    builder.build();
    var error = assertThrows(IllegalStateException.class, () -> builder.add(29, "FrontLeft turn"));
    assertTrue(error.getMessage().contains("FrontLeft turn (ID 29)"));
  }

  @Test
  void builtChainCannotBeModified() {
    var builder = new CANChainBuilder();
    builder.add(28, "FrontLeft drive");
    var chain = builder.build();
    assertThrows(
        UnsupportedOperationException.class, () -> chain.add(new CANChainDevice(1, "extra")));
  }
}
