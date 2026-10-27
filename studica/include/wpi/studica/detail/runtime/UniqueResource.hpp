// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <functional>
#include <optional>
#include <utility>

namespace wpi::studica::detail {
// Ownership is independent of the handle's value: zero can be a valid handle.
template <class Handle, class Releaser = std::function<void(Handle)>>
class UniqueResource {
 public:
  UniqueResource() = default;
  UniqueResource(Handle handle, Releaser release)
      : m_release{std::move(release)}, m_handle{handle} {}
  UniqueResource(const UniqueResource&) = delete;
  UniqueResource& operator=(const UniqueResource&) = delete;
  UniqueResource(UniqueResource&& other) noexcept
      : m_release{std::move(other.m_release)},
        m_handle{std::exchange(other.m_handle, std::nullopt)} {}
  UniqueResource& operator=(UniqueResource&& other) noexcept {
    if (this != &other) {
      Reset();
      m_release = std::move(other.m_release);
      m_handle = std::exchange(other.m_handle, std::nullopt);
    }
    return *this;
  }
  ~UniqueResource() { Reset(); }
  Handle Get() const { return m_handle.value(); }
  void Reset() noexcept {
    if (auto handle = std::exchange(m_handle, std::nullopt)) {
      try {
        m_release(*handle);
      } catch (...) { /* Destructors cannot throw. */
      }
    }
  }

 private:
  Releaser m_release{};
  std::optional<Handle> m_handle;
};
}  // namespace wpi::studica::detail
