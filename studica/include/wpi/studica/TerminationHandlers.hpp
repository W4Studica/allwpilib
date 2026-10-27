// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace wpi::studica {

/**
 * Makes SIGINT and SIGTERM end the process after the handler that is installed now has run.
 *
 * The VMX HAL (pigpio) installs handlers for SIGINT and SIGTERM that release the hardware and then return. A WPILib
 * desktop program has no handler of its own and normally dies of the default action, so on a VMX it never stopped:
 * robot_manager waited its whole grace period and killed it (every deploy took 5 s longer). MEASURED on a VMX-pi:
 * after "Exiting VMX-pi HAL application ... [normal exit]" the process kept running.
 *
 * The VMX HAL may install its handlers after the extension started (MEASURED: a handler installed once at start was never
 * called), so call this repeatedly: it does nothing for a signal whose handler is already ours, and otherwise installs
 * ours in front of whatever is there now. HALSimStudica does it on every poll. For each of SIGINT and SIGTERM that
 * already has a handler, it installs one that calls the previous handler (the hardware clean-up), then restores the
 * default action and re-raises the signal, so the process ends like any other desktop robot program. A clean-up that hangs
 * is cut short after one second (SIGALRM). Signals without a
 * handler (a PC simulation with the loopback backend) are left alone. Does nothing on Windows.
 */
void InstallTerminationHandlers();

}  // namespace wpi::studica
