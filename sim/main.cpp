/*
  Native mms simulator entry point.
  -----------------------------------------------------------------
  Same Mouse core as the firmware; only the MouseIO bridge differs.

  Build:  pio run -e sim
  Binary: .pio/build/sim/program

  Logs are written to stderr and appear in mms under "Run Output".
  Set the level to LogLevel::Debug for a full protocol + decision trace.
*/

#include <chrono>
#include <thread>

#include "mouse.h"
#include "sim_io.h"

int main() {
  SimIO io;
  io.setLogLevel(LogLevel::Debug);

  Mouse mouse(io);

  for (;;) {
    mouse.reset();
    // run() drives the full lifecycle: explore to the goal, return to the
    // start mapping fresh corridors, keep re-exploring while an unknown route
    // could still beat the best known path, then commit to a speed run.
    mouse.run();

    // Idle until the simulator's reset button is pressed.
    while (!io.resetRequested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    io.resetAck();
  }

  return 0;
}
