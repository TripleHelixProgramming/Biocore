# AGENTS.md

Instructions for coding agents that build, test, and simulate this robot project.
For people, [README.md](README.md) and [doc/CONTRIBUTING.md](doc/CONTRIBUTING.md) cover the same ground.

## Toolchain

- GradleRIO `2027.0.0-alpha-7`, Java source and target level 25 (see [build.gradle](build.gradle)).
- Use the JDK that ships with the WPILib 2027 alpha-7 installer. On Windows the default
  location is `C:\Users\Public\wpilib\2027_alpha7\jdk`. Point `JAVA_HOME` at it before you
  run Gradle:

  ```powershell
  $env:JAVA_HOME = "C:\Users\Public\wpilib\2027_alpha7\jdk"
  ```

  ```bash
  export JAVA_HOME="/c/Users/Public/wpilib/2027_alpha7/jdk"
  ```

- An older JDK (for example, a system Java 17) causes these failures:
  - `compileJava`: `error: invalid source release: 25`.
  - `spotlessJavaApply`: `NoClassDefFoundError ... JCTree$JCAnyPattern` on every Java file.
- Spotless keeps a failed format result after you change the JDK. After you set `JAVA_HOME`
  correctly, clear the stored result once:

  ```bash
  ./gradlew spotlessJava --rerun
  ```

  `--rerun` applies only to the task that comes immediately before it. It must also run under
  the correct JDK, or it stores the same failure again.

## Build

```bash
./gradlew build
```

- `compileJava` depends on `spotlessApply`, so every build reformats source files in place.
  Expect formatting diffs after a build.
- Tests run as part of `build` with `ignoreFailures = true`. A test failure does not fail the
  build. Search the output for `WARNING: N TEST(S) FAILED!` and read the report at
  `build/reports/tests/test/index.html`.

Run one test class:

```bash
./gradlew test --tests "*VisionFilterTest*"
```

## Simulate

The robot runs in simulation through the application plugin's `run` task:

```bash
./gradlew run
```

- By default the simulation is headless: the log shows `HAL Extensions: No extensions found`.
  The robot is ready when the log shows `Robot program startup complete`.
- The process runs until you stop it. From a script, stop it with Ctrl+C or end the `java`
  process whose command line contains `frc.robot.Main`.
- AdvantageKit writes `.wpilog` files to `logs/`. Git ignores that directory.

To open the WPILib sim GUI, load its HAL extension with the `HALSIM_EXTENSIONS` environment
variable. Run `./gradlew build` first so that the extension library exists in
`build/jni/release/`:

```powershell
$env:HALSIM_EXTENSIONS = "$PWD\build\jni\release\halsim_gui.dll"
./gradlew run
```

On Linux the file is `libhalsim_gui.so`. On macOS it is `libhalsim_gui.dylib`. To use a real
Driver Station, load `halsim_ds_socket` in the same way. Separate more than one library
with `;` on Windows or `:` on Linux and macOS.

In VS Code, **WPILib: Simulate Robot Code** reads `build/sim/java.json` and shows a checkbox
for each extension.

### Sim or replay

`Constants.simMode` in
[src/main/java/frc/robot/Constants.java](src/main/java/frc/robot/Constants.java) selects
what a non-roboRIO run does:

- `RobotMode.SIM`: physics simulation.
- `RobotMode.REPLAY`: AdvantageKit log replay. The `replayWatch` Gradle task re-runs replay
  when code changes.

Feature flags such as `VISION_ENABLED` and `LEDS_ENABLED` are in the same file.

## Deploy

```bash
./gradlew deploy
```

This deploys to a SystemCore. The team number comes from `.wpilib/wpilib_preferences.json`.
On a branch whose name starts with `event`, a deploy first commits all working changes
(`git add -A`) with a timestamp message.
