#include "sim_io.h"

#include <chrono>
#include <cstdio>
#include <iostream>

namespace {
const char kDirChar[4] = {'n', 'e', 's', 'w'};

double elapsedSeconds() {
  using namespace std::chrono;
  static const auto start = steady_clock::now();
  return duration<double>(steady_clock::now() - start).count();
}
} // namespace

void SimIO::emitLog(LogLevel level, const char* message) {
  char prefix[40];
  std::snprintf(prefix, sizeof(prefix), "[%8.3f][%-5s] ", elapsedSeconds(),
                logLevelName(level));
  std::cerr << prefix << message << std::endl;
}

// Protocol commands that expect a single response line.
std::string SimIO::send(const std::string& command) {
  std::cout << command << std::endl;

  std::string response;
  std::getline(std::cin, response);

  // Do not trace the reset poll, otherwise the idle loop floods the log.
  if (command != "wasReset" && logEnabled(LogLevel::Debug)) {
    char buffer[160];
    std::snprintf(buffer, sizeof(buffer), "mms -> %s | <- %s", command.c_str(),
                  response.c_str());
    log(LogLevel::Debug, buffer);
  }
  return response;
}

// Protocol commands that produce no response.
void SimIO::sendCommand(const std::string& command) {
  std::cout << command << std::endl;

  if (logEnabled(LogLevel::Debug)) {
    std::string message = "mms -> " + command;
    log(LogLevel::Debug, message.c_str());
  }
}

bool SimIO::query(const std::string& command) { return send(command) == "true"; }

bool SimIO::wallFront() { return query("wallFront"); }
bool SimIO::wallRight() { return query("wallRight"); }
bool SimIO::wallLeft() { return query("wallLeft"); }

void SimIO::moveForward() {
  std::string response = send("moveForward");
  if (response == "crash") {
    log(LogLevel::Error, "mms reported a crash on moveForward");
  }
}

void SimIO::turnRight() { send("turnRight"); }
void SimIO::turnLeft() { send("turnLeft"); }

void SimIO::showWall(int x, int y, int dir) {
  sendCommand("setWall " + std::to_string(x) + " " + std::to_string(y) + " " +
              kDirChar[dir]);
}

void SimIO::showText(int x, int y, const char* text) {
  sendCommand("setText " + std::to_string(x) + " " + std::to_string(y) + " " +
              text);
}

bool SimIO::resetRequested() { return query("wasReset"); }
void SimIO::resetAck() { send("ackReset"); }
