// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.auto;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;
import java.util.TreeMap;
import java.util.stream.Stream;

/** Reads the Choreo project and trajectory files straight from the deploy directory. */
public final class ChoreoProjectFiles {
  public static final Path CHOREO_DIR = Path.of("src", "main", "deploy", "choreo");

  private ChoreoProjectFiles() {}

  /** Returns the parsed {@code Rho.chor} project file. */
  public static JsonObject project() {
    return read(CHOREO_DIR.resolve("Rho.chor"));
  }

  /** Returns every parsed {@code .traj} file, keyed by trajectory name. */
  public static Map<String, JsonObject> trajectories() {
    Map<String, JsonObject> trajectories = new TreeMap<>();
    try (Stream<Path> files = Files.list(CHOREO_DIR)) {
      files
          .filter(p -> p.toString().endsWith(".traj"))
          .forEach(p -> trajectories.put(p.getFileName().toString().replace(".traj", ""), read(p)));
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    return trajectories;
  }

  /** Returns true if the element is a Choreo expression: an object with exactly exp and val. */
  public static boolean isExpr(JsonElement element) {
    if (!element.isJsonObject()) return false;
    JsonObject object = element.getAsJsonObject();
    return object.size() == 2 && object.has("exp") && object.has("val");
  }

  /** Returns the evaluated value of a Choreo expression. */
  public static double val(JsonElement expr) {
    return expr.getAsJsonObject().get("val").getAsDouble();
  }

  /**
   * Returns a copy of the element with every Choreo expression replaced by its value. This is the
   * shape Choreo stores in a trajectory's snapshot and robot config.
   */
  public static JsonElement values(JsonElement element) {
    if (isExpr(element)) return element.getAsJsonObject().get("val");
    if (element.isJsonObject()) {
      JsonObject copy = new JsonObject();
      element.getAsJsonObject().entrySet().forEach(e -> copy.add(e.getKey(), values(e.getValue())));
      return copy;
    }
    if (element.isJsonArray()) {
      JsonArray copy = new JsonArray();
      element.getAsJsonArray().forEach(e -> copy.add(values(e)));
      return copy;
    }
    return element;
  }

  private static JsonObject read(Path path) {
    try {
      return JsonParser.parseString(Files.readString(path)).getAsJsonObject();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }
}
