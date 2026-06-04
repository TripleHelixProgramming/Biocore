# Biocore — FRC 2363 2027 Robot Code
[![CI](https://github.com/TripleHelixProgramming/Biocore/actions/workflows/build.yml/badge.svg)](https://github.com/TripleHelixProgramming/Biocore/actions/workflows/build.yml)

Java robot code for the 2027 FRC season, built on [WPILib](https://docs.wpilib.org) and [AdvantageKit](https://docs.advantagekit.org/).

## Contributing

See [How to Contribute](doc/CONTRIBUTING.md) for the full workflow: environment setup, branching, conflict resolution, formatting, and PR review requirements.

---

## Documentation

### Architecture

| Doc | What it covers |
|-----|---------------|
| [Drive Subsystem: Pose and Heading](doc/DRIVE_HEADING_REFACTOR.md) | Why there is one rotation source of truth, how `Drive` and `DriveCommands` divide responsibilities, what `setPose` and `getHeading` do |
| [I/O Architecture: Background Threading](doc/TWEAKS.md) | Why sensor reads happen on background threads, how Phoenix 6 auto-refresh works, deferred logging pattern, profiling infrastructure |
| [Defensive Guards](doc/DEFENSIVE_GUARDS.md) | Philosophy for numerical guard placement (NaN, division by zero), where guards exist and why |

### Subsystems

| Doc | What it covers |
|-----|---------------|
| [LED System](doc/LED.md) | Physical strip layout, series/portion definitions, display functions, how to add custom patterns |
| [Pneumatics Simulation](doc/pneumatics-sim.md) | Physics model behind `PneumaticsSimulator` — ideal gas law, Cv flow, compressor model, piston dynamics |

### Vision

| Doc | What it covers |
|-----|---------------|
| [Vision Calibration and Validation](doc/VISION_CALIBRATION_AND_VALIDATION.md) | Camera intrinsics, extrinsics, field layout accuracy, pre-season and per-event validation procedures |
| [VisionFilter Architecture](doc/VISION_TESTS.md) | How observations are scored and filtered — weighted geometric mean, velocity consistency, cross-camera correlation boost |
| [Vision Filter Tuning](doc/VISION_FILTER_TUNING.md) | Why `minScore = 0.6` and `velocityUncertainScore = 0.7`, how to check the score distribution in logs |

### Autonomous

| Doc | What it covers |
|-----|---------------|
| [Improving Trajectory Tracking](doc/TRAJECTORY_TRACKING_IMPROVEMENTS.md) | Where tracking error comes from and a tiered menu of strategies to reduce it |

---

## Requirements

- [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) (includes Java 17 and VS Code extensions)
- A roboRIO-connected robot or simulation environment

---

## Building

Use the Gradle wrapper — no separate Gradle installation needed.

**Build (compile + check):**
```bash
./gradlew build
```

**Deploy to robot:**
```bash
./gradlew deploy
```

**Run in simulation:**
```bash
./gradlew simulateJava
```

---

## Code Formatting

This project uses [Spotless](https://github.com/diffplug/spotless) with Google Java Format. Formatting is applied automatically on every build.

**Apply formatting manually:**
```bash
./gradlew spotlessApply
```

**Check formatting without modifying files:**
```bash
./gradlew spotlessCheck
```

---

## AdvantageScope Custom Assets

The `ascope-assets/` directory contains custom robot models and camera configurations for [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope).

**To enable them in AdvantageScope:**

1. Open AdvantageScope.
2. Go to **Help → Show App Directory** (or press `Cmd/Ctrl+Shift+.`).
3. In your AdvantageScope settings (or via **File → Preferences**), set the **Custom Assets** folder to the `ascope-assets/` directory in this repository.
   - Example path: `/path/to/Rebuilt/ascope-assets`
4. Restart AdvantageScope. The robot model and camera views will appear in the 3D field viewer.

---

## Utility Controls

### Align Encoders

Resets the swerve drive absolute encoders. This can be triggered while the robot is disabled.

1. Open [Glass](https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/index.html).
2. Go to **NetworkTables → Triggers → Align Encoders**.
3. Toggle the value to `true`.

---

## Simulation

### Keyboard Driver

When running in simulation, a keyboard (`Keyboard 0`) can be used as a driver controller. WASD controls translation and axis 2 controls rotation. Z resets heading.

**Configuring rotation (axis 2) for left/right arrow keys:**

> **Note:** These settings are not persistent and must be re-applied each time the simulator is opened.

1. Open the sim Driver Station.
2. Go to **DS → Keyboard 0 settings**.
3. Update the axis 2 bindings:

| Setting | Default | Change to |
|---|---|---|
| Increase key | `e` | Right arrow |
| Decrease key | `r` | Left arrow |
| Key rate | `0.01` | `0.050` |
| Decay rate | `0` | `0.050` |
| Max absolute value | `1.0` | `1.0` |

---

## Credits and Licensing

This project is licensed under the [BSD 3-Clause License](LICENSE). See individual source files for copyright holders.

### Triple Helix Robotics (FRC Team 2363)

The robot-specific subsystems (launcher, feeder, intake, hopper, LEDs, autos, and supporting utilities) are original work by Triple Helix Robotics, copyright 2025.

### AdvantageKit — Littleton Robotics (FRC 6328)

The swerve drive subsystem, I/O architecture, and logging infrastructure are based on the [AdvantageKit TalonFX Swerve template](https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/template_projects/sources/talonfx_swerve) by Littleton Robotics (FRC 6328 "Mechanical Advantage"), and have been substantially modified. AdvantageKit is licensed under BSD 3-Clause; see [AdvantageKit-License.md](AdvantageKit-License.md).

### WPILib

This project uses [WPILib](https://docs.wpilib.org), the standard FRC robot programming library maintained by FIRST and WPILib contributors, licensed under BSD; see [WPILib-License.md](WPILib-License.md).
