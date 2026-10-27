// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <cstdio>
#include <memory>

#include "VMXPi.h"
#include "wpi/studica/detail/runtime/UniqueResource.hpp"

namespace wpi::studica::detail {
struct VMXRelease {
  std::shared_ptr<VMXPi> vmx;
  void (*beforeRelease)(VMXPi&, VMXResourceHandle, uintptr_t) = nullptr;
  uintptr_t token = 0;
  void operator()(VMXResourceHandle resource) const noexcept {
    if (beforeRelease)
      beforeRelease(*vmx, resource, token);
    VMXErrorCode error = 0;
    if (!vmx->io.DeallocateResource(resource, &error)) {
      std::fprintf(stderr, "Studica: resource release failed: %s\n",
                   GetVMXErrorString(error));
    }
  }
};
using VMXResource = UniqueResource<VMXResourceHandle, VMXRelease>;
// No allocation between SDK acquisition and RAII ownership.
inline VMXResource OwnVMXResource(std::shared_ptr<VMXPi> vmx,
                                  VMXResourceHandle handle) noexcept {
  return {handle, VMXRelease{std::move(vmx)}};
}
}  // namespace wpi::studica::detail
