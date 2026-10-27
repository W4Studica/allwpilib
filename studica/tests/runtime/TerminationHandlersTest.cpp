// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/detail/runtime/TerminationHandlers.hpp"

#include <chrono>

#include <catch2/catch_test_macros.hpp>

#if !defined(_WIN32)

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

int gPipeFd = -1;

void PreviousHandler(int) {
  const char byte = 'x';
  [[maybe_unused]]
  auto ignored = write(gPipeFd, &byte, 1);
}

// Runs `body` in a child process; returns its wait status and whether the
// previous handler ran.
template <typename F>
int RunInChild(F body, bool* handlerRan) {
  int fds[2];
  REQUIRE(pipe(fds) == 0);
  pid_t pid = fork();
  REQUIRE(pid >= 0);
  if (pid == 0) {
    close(fds[0]);
    gPipeFd = fds[1];
    body();
    _exit(42);  // reached only if the signal did not end the process
  }
  close(fds[1]);
  int status = 0;
  REQUIRE(waitpid(pid, &status, 0) == pid);
  char byte = 0;
  *handlerRan = read(fds[0], &byte, 1) == 1;
  close(fds[0]);
  return status;
}

}  // namespace

TEST_CASE("SIGTERM runs the previous handler, then ends the process",
          "[halsim_studica]") {
  bool ran = false;
  int status = RunInChild(
      [] {
        signal(SIGTERM, PreviousHandler);
        wpi::studica::InstallTerminationHandlers();
        raise(SIGTERM);
        pause();
      },
      &ran);
  REQUIRE(WIFSIGNALED(status));
  REQUIRE(WTERMSIG(status) == SIGTERM);
  REQUIRE(ran);
}

TEST_CASE("SIGINT runs the previous handler, then ends the process",
          "[halsim_studica]") {
  bool ran = false;
  int status = RunInChild(
      [] {
        signal(SIGINT, PreviousHandler);
        wpi::studica::InstallTerminationHandlers();
        raise(SIGINT);
        pause();
      },
      &ran);
  REQUIRE(WIFSIGNALED(status));
  REQUIRE(WTERMSIG(status) == SIGINT);
  REQUIRE(ran);
}

TEST_CASE("a signal without a handler keeps its default action",
          "[halsim_studica]") {
  bool ran = false;
  int status = RunInChild(
      [] {
        signal(SIGTERM, SIG_DFL);
        wpi::studica::InstallTerminationHandlers();
        raise(SIGTERM);
        pause();
      },
      &ran);
  REQUIRE(WIFSIGNALED(status));
  REQUIRE(WTERMSIG(status) == SIGTERM);
  REQUIRE_FALSE(ran);
}

void HangingHandler(int) {
  for (;;) {
    pause();  // the VMX HAL clean-up stuck inside exit()
  }
}

TEST_CASE("a previous handler that hangs is cut short by SIGALRM",
          "[halsim_studica]") {
  bool ran = false;
  auto start = std::chrono::steady_clock::now();
  int status = RunInChild(
      [] {
        signal(SIGTERM, HangingHandler);
        wpi::studica::InstallTerminationHandlers();
        raise(SIGTERM);
        pause();
      },
      &ran);
  auto seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  REQUIRE(WIFSIGNALED(status));
  REQUIRE(WTERMSIG(status) == SIGALRM);
  REQUIRE(seconds < 3.0);
}

TEST_CASE(
    "a handler installed after InstallTerminationHandlers is taken over on the "
    "next call",
    "[halsim_studica]") {
  bool ran = false;
  int status = RunInChild(
      [] {
        wpi::studica::InstallTerminationHandlers();  // no handler yet: nothing
                                                     // happens
        signal(SIGTERM, PreviousHandler);  // the VMX HAL installs its own later
        wpi::studica::InstallTerminationHandlers();
        wpi::studica::InstallTerminationHandlers();  // already ours: must not
                                                     // chain to itself
        raise(SIGTERM);
        pause();
      },
      &ran);
  REQUIRE(WIFSIGNALED(status));
  REQUIRE(WTERMSIG(status) == SIGTERM);
  REQUIRE(ran);
}

TEST_CASE("an ignored signal stays ignored", "[halsim_studica]") {
  bool ran = false;
  int status = RunInChild(
      [] {
        signal(SIGTERM, SIG_IGN);
        wpi::studica::InstallTerminationHandlers();
        raise(SIGTERM);
      },
      &ran);
  REQUIRE(WIFEXITED(status));
  REQUIRE(WEXITSTATUS(status) == 42);
}

#endif
