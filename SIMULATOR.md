# Running the Simulator (mms)

This project shares one maze-solving core between the real robot and the
[mackorone/mms](https://github.com/mackorone/mms) simulator. Only the bridge
(adapter) changes.

```
include/mouse.h        MouseIO interface + Mouse core (platform-agnostic)
src/mouse.cpp          flood fill / navigation (identical for both targets)

src/hardware_io.cpp    HardwareIO bridge -> IR sensors + L293D  (firmware)
sim/sim_io.cpp         SimIO bridge      -> mms stdin/stdout   (host)

src/main.cpp           firmware entry  (uses HardwareIO)
sim/main.cpp           simulator entry (uses SimIO)
```

`MouseIO` is the seam between the algorithm and the hardware:

| Method | Meaning |
| :--- | :--- |
| `wallFront()`, `wallRight()`, `wallLeft()` | cardinal walls relative to the current heading |
| `wallDiagonalLeft()`, `wallDiagonalRight()` | 45-degree forward-corner sensors (default derives them from the cardinal pair) |
| `moveForward()`, `turnRight()`, `turnLeft()` | one cell / 90 degrees |
| `showWall()`, `showText()` | optional visualization (no-op on hardware) |
| `resetRequested()`, `resetAck()` | optional crash/reset handling (simulator only) |

The algorithm in `src/mouse.cpp` never includes `<Arduino.h>` or any simulator
header, so it builds for both.

### Sensor fan

The robot carries six analog IR sensors at `-90, -45, 0, 0, +45, +90` degrees
(see `include/config.h`). The 45-degree pair looks at the forward corners, so a
reading there that the side sensor does not share is treated as a front wall —
this catches walls the two 0-degree sensors miss. The core also logs the local
junction type (`corridor`, `open-left`, `dead-end`, `cross`, ...).

The mms simulator only models cardinal walls, so `SimIO` inherits the default
diagonal derivation; on hardware `HardwareIO` overrides them with the real
sensors.

## Prerequisites

- **PlatformIO Core.** The VS Code PlatformIO extension already installs the
  CLI. It lives at `~/.platformio/penv/bin/pio`. Add it to your `PATH` by
  putting this in `~/.zshrc`:

  ```sh
  export PATH="$HOME/.platformio/penv/bin:$PATH"
  ```

  Then reload: `source ~/.zshrc` and verify with `pio --version`.

- **mms.** Download a release from
  <https://github.com/mackorone/mms/releases> and run the app. On macOS, if you
  see *"mms.app is damaged"*, clear the quarantine flag:

  ```sh
  xattr -d com.apple.quarantine mms.app
  ```

## Build the simulator

From the project root:

```sh
pio run -e sim
```

This produces a native host binary (not STM32 firmware) at:

```
.pio/build/sim/program
```

> Rebuild with this command after **any** change to `src/mouse.cpp`,
> `sim/sim_io.cpp` or `sim/main.cpp`, otherwise mms keeps running the old
> binary.

## Configure mms

Click the **+** button in mms and fill in the fields. Use **absolute paths** —
mms is a GUI app and does *not* inherit the `PATH` from your shell, so it cannot
find `pio` (or a relative `program`) on its own.

| Field | Value |
| :--- | :--- |
| **Name** | `Micromouse` |
| **Directory** | `/Volumes/Projects/uni-projects/micromouse` |
| **Build Command** | `/Users/dulranga/.platformio/penv/bin/pio run -e sim` |
| **Run Command** | `/Volumes/Projects/uni-projects/micromouse/.pio/build/sim/program` |

Substitute your own project location and home directory. Then:

1. Click **Build** (compiles the native binary).
2. Click **Run** (launches it and connects to the simulator).

## Logging

The shared core emits a structured trace through the `MouseIO` bridge, and
each bridge decides where it goes. **stdout is reserved for the mms protocol**,
so the simulator bridge writes all logs to **stderr**, which mms shows in the
**Run Output** tab.

Log levels (`LogLevel` in `include/mouse.h`), from most to least verbose:

| Level | Content |
| :--- | :--- |
| `Debug` | every mms command/response, wall detections, flood-fill distance, per-step state |
| `Info` | reset, each move (from/to, action, distance), goal summary |
| `Warn` | recoverable problems |
| `Error` | e.g. mms reporting a crash |
| `None` | silent |

Set the level on the bridge:

```cpp
io.setLogLevel(LogLevel::Debug); // sim/main.cpp (full trace)
io.setLogLevel(LogLevel::Info);  // src/main.cpp (on-board Serial)
```

Example simulator trace:

```
[   0.000][INFO ] reset: pos=(0,0) heading=N goal=(7..8,7..8)
[   0.000][DEBUG] step 1 [EXPLORE->GOAL]: pos=(0,0) heading=N
[   0.000][DEBUG] mms -> wallFront | <- false
[   0.000][DEBUG] sense: pos=(0,0) heading=N front=0 left=1 right=0 ...
[   0.000][DEBUG] wall added: (0,0) W
[   0.000][DEBUG] flood(goal,optimistic): cost(0,0)=30
[   0.000][INFO ] move 1: (0,0) N -> (0,1) N [forward, cost=27]
...
[   0.003][INFO ] GOAL reached: moves=14 turns=1 walls=8
[   0.006][INFO ] START reached: moves=28 turns=3 walls=15
[   0.006][INFO ] explore-check: knownBest=29 optimisticBest=29 -> commit
[   0.006][INFO ] committing to speed run (optimal known path)
[   0.009][INFO ] SPEED RUN complete: moves=42 turns=5
```

The run now has multiple phases: it explores out to the goal (treating
unknown cells as open), returns to the start mapping fresh corridors, keeps
re-exploring while an unknown route could still beat the best fully-known
path, then commits to a turn-penalized optimal speed run.

On hardware, `HardwareIO` writes the same messages to `Serial` (115200). At
`Debug` it also logs every IR reading (`IR front: 42 51 cm`, etc.), which is
useful for tuning thresholds.

To silence logging without changing code, use `LogLevel::None`.

## Troubleshooting

**`Child process set up failed: execve: No such file or directory`**

The command mms tried to run does not exist. Check:

- The binary was built: `ls .pio/build/sim/program`.
- The Run Command points at that file with an **absolute** path.
- The Build Command uses an **absolute** path to `pio`
  (`.../.platformio/penv/bin/pio`), not just `pio`. If it still fails, run
  `pio run -e sim` manually in a terminal and set Build Command to blank.

**mms can't find `pio`**

Same cause: GUI apps don't read `~/.zshrc`. Always use the full path to `pio` in
the Build Command.

**Algorithm never reaches the goal / crashes**

The maze is 16x16 with a central 2x2 goal at cells `(7..8, 7..8)` (0-based),
configured by `Mouse::GOAL_X0/X1/Y0/Y1` in `include/mouse.h`. Adjust those
constants if your maze differs.

## Adding another bridge

To target a new platform, subclass `MouseIO` and implement the six motion/wall
methods, then build the shared `Mouse` core against it. No changes to the
algorithm are required.
