// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <mutex>

#include "../StudicaBackend.hpp"

namespace wpi::studica {
namespace {
using detail::InterruptQueue;
// The SDK retains an opaque token, never a pointer to backend-owned memory.
// Late callbacks cannot access a destroyed backend, even while a pin is being
// freed.
std::mutex gInterruptMutex;
std::map<uintptr_t, std::shared_ptr<InterruptQueue>> gInterruptQueues;
uintptr_t gNextInterruptToken = 1;
void OnInterrupt(uint32_t, InterruptEdgeType edge, void* param,
                 uint64_t timestamp) {
  std::shared_ptr<InterruptQueue> queue;
  {
    std::scoped_lock lock{gInterruptMutex};
    auto it = gInterruptQueues.find(reinterpret_cast<uintptr_t>(param));
    if (it == gInterruptQueues.end()) {
      return;
    }
    queue = it->second;
  }
  queue->Push({edge == InterruptEdgeType::RISING_EDGE_INTERRUPT, timestamp, 0});
}
}  // namespace

bool StudicaBackend::InitInterrupt(int channel) {
  if (!m_vmx || !m_vmx->IsOpen() || ChannelBusy(channel) ||
      !ChannelCan(channel, VMXChannelCapability::InterruptInput)) {
    return false;
  }
  Interrupt interrupt;
  interrupt.queue = std::make_shared<InterruptQueue>();
  {
    std::scoped_lock lock{gInterruptMutex};
    if (gNextInterruptToken == 0) {
      return false;
    }
    interrupt.token = gNextInterruptToken++;
    gInterruptQueues.emplace(interrupt.token, interrupt.queue);
  }
  InterruptConfig config(InterruptConfig::BOTH, OnInterrupt,
                         reinterpret_cast<void*>(interrupt.token));
  VMXErrorCode err = 0;
  VMXResourceHandle handle{};
  if (!m_vmx->io.ActivateSinglechannelResource(
          VMXChannelInfo(channel, VMXChannelCapability::InterruptInput),
          &config, handle, &err)) {
    std::scoped_lock lock{gInterruptMutex};
    gInterruptQueues.erase(interrupt.token);
    return false;
  }
  interrupt.resource = detail::VMXResource{
      handle, detail::VMXRelease{
                  m_vmx,
                  [](VMXPi& vmx, VMXResourceHandle resource, uintptr_t token) {
                    {
                      std::scoped_lock lock{gInterruptMutex};
                      gInterruptQueues.erase(token);
                    }
                    VMXErrorCode error = 0;
                    vmx.io.Interrupt_SetEnabled(resource, false, &error);
                  },
                  interrupt.token}};
  m_interrupts.emplace(channel, std::move(interrupt));
  return true;
}

void StudicaBackend::ReleaseInterrupt(int channel) {
  auto it = m_interrupts.find(channel);
  if (it == m_interrupts.end()) {
    return;
  }
  m_interrupts.erase(it);
}

bool StudicaBackend::ReadInterrupt(int channel, InterruptEvent* event) {
  auto it = m_interrupts.find(channel);
  if (it == m_interrupts.end()) {
    return false;
  }
  return it->second.queue->Pop(event);
}
}  // namespace wpi::studica
