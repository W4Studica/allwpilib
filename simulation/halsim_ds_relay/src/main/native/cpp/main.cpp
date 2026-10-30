// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "wpi/hal/Extensions.h"
#include "wpi/hal/simulation/RoboRioData.h"
#include "wpi/halsim/ds_relay/DsRelayFrames.hpp"

using namespace wpi::halsim::ds_relay;

namespace {

/// Poll period for the receive loop.
constexpr auto kPollPeriod = std::chrono::milliseconds(10);
/// How long the relay waits for a retry when the bind fails.
constexpr auto kRetryBindPeriod = std::chrono::seconds(1);
/// Defensive watchdog: apply "disabled + not attached" after this silence.
constexpr auto kWatchdogTimeout = std::chrono::seconds(1);

/// Owns the poll thread so it is always stopped before static destruction.
struct Relay {
  std::atomic<bool> stop{false};
  std::thread thread;

  ~Relay() {
    stop.store(true, std::memory_order_relaxed);
    if (thread.joinable()) {
      thread.join();
    }
  }
};

Relay gRelay;

/**
 * Creates and binds a non-blocking UDP socket on 127.0.0.1:port.
 * Returns the file descriptor, or -1 on failure.
 */
int TryBind(int port) {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    return -1;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    return -1;
  }
  int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  }
  return fd;
}

void PollLoop(int port) {
  int fd = -1;
  bool warned = false;
  uint32_t statusCount = 0;

  uint8_t recvBuf[kStateFrameSize];
  uint8_t statusBuf[kStatusFrameSize];
  sockaddr_in sender{};
  socklen_t senderLen = 0;

  bool everReceived = false;
  bool watchdogApplied = false;
  auto lastReceive = std::chrono::steady_clock::now();

  while (!gRelay.stop.load(std::memory_order_relaxed)) {
    if (fd < 0) {
      fd = TryBind(port);
      if (fd < 0) {
        if (!warned) {
          std::printf(
              "HALSim DS Relay: could not bind 127.0.0.1:%d, retrying every "
              "second\n",
              port);
          warned = true;
        }
        std::this_thread::sleep_for(kRetryBindPeriod);
        continue;
      }
      warned = false;
      lastReceive = std::chrono::steady_clock::now();
      std::printf("HALSim DS Relay: listening on 127.0.0.1:%d\n", port);
    }

    sockaddr_in from{};
    socklen_t fromLen = sizeof(from);
    ssize_t received =
        ::recvfrom(fd, recvBuf, sizeof(recvBuf), 0,
                   reinterpret_cast<sockaddr*>(&from), &fromLen);
    if (received > 0) {
      DsState state;
      if (DecodeStateFrame(recvBuf, static_cast<size_t>(received), state)) {
        ApplyStateFrame(state);
        sender = from;
        senderLen = fromLen;
        everReceived = true;
        watchdogApplied = false;
        lastReceive = std::chrono::steady_clock::now();

        // Answer the newest valid sender with the current battery voltage.
        size_t statusSize = EncodeStatusFrame(
            /*robotProgramAlive=*/true,
            static_cast<float>(HALSIM_GetRoboRioVInVoltage()), statusCount,
            statusBuf);
        if (statusSize == kStatusFrameSize) {
          ::sendto(fd, statusBuf, statusSize, 0,
                   reinterpret_cast<sockaddr*>(&sender), senderLen);
        }
        ++statusCount;
      }
      // Invalid frames with the wrong magic/version/size are ignored silently.
    } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
               errno != EINTR) {
      // Socket error: drop it and rebind on the next iteration.
      ::close(fd);
      fd = -1;
      continue;
    }

    if (everReceived && !watchdogApplied &&
        std::chrono::steady_clock::now() - lastReceive > kWatchdogTimeout) {
      ApplyWatchdogDisconnected();
      watchdogApplied = true;
    }

    std::this_thread::sleep_for(kPollPeriod);
  }

  if (fd >= 0) {
    ::close(fd);
  }
}

void StopRelay() {
  gRelay.stop.store(true, std::memory_order_relaxed);
  if (gRelay.thread.joinable()) {
    gRelay.thread.join();
  }
}

}  // namespace

extern "C" {
#if defined(WIN32) || defined(_WIN32)
__declspec(dllexport)
#endif

int HALSIM_InitExtension(void) {
  std::puts("HALSim DS Relay Extension Initializing");

  int port = GetRelayPort();

  HAL_OnShutdown(nullptr, [](void*) { StopRelay(); });

  // Never leave a joinable thread behind if init is invoked again.
  StopRelay();

  gRelay.stop.store(false, std::memory_order_relaxed);
  gRelay.thread = std::thread(PollLoop, port);

  std::printf(
      "HALSim DS Relay: relaying Driver Station state on 127.0.0.1:%d\n", port);
  std::puts("HALSim DS Relay Extension Initialized");
  return 0;
}

}  // extern "C"
