// Prints the VMX channel map the HAL reports: channel indexes by type and what each channel can do.
// Use it to choose the VMX channel numbers for HALSIMVMX_DIO_MAP / HALSIMVMX_ANALOG_MAP instead of guessing.
//
//   g++ -std=c++17 vmx_channels.cpp -o vmx_channels -I/usr/local/include/vmxpi -L/usr/local/lib/vmxpi -lvmxpi_hal_cpp -lrt -lpthread -latomic
//   sudo ./vmx_channels
//
// Opens the VMX once (one VMXPi), reads the channel table, and closes it. It drives no outputs.

#include <cstdint>
#include <cstdio>
#include <string>

#include "VMXPi.h"

static const char* TypeName(VMXChannelType t) {
  switch (t) {
    case FlexDIO:
      return "FlexDIO";
    case AnalogIn:
      return "AnalogIn";
    case HiCurrDIO:
      return "HiCurrDIO";
    case CommDIO:
      return "CommDIO";
    default:
      return "?";
  }
}

static std::string Capabilities(uint32_t bits) {
  struct Flag {
    uint32_t bit;
    const char* name;
  };
  static const Flag flags[] = {
      {0x00000001, "DigitalIn"},   {0x00000002, "DigitalOut"},  {0x00000004, "PWMGen"},
      {0x00000008, "PWMGen2"},     {0x00000010, "PWMCap"},      {0x00000020, "PWMCap2"},
      {0x00000040, "EncA"},        {0x00000080, "EncB"},        {0x00000100, "Accum"},
      {0x00000200, "AnalogTrig"},  {0x00000400, "Interrupt"},   {0x00000800, "UART_TX"},
      {0x00001000, "UART_RX"},     {0x00002000, "SPI_CLK"},     {0x00004000, "SPI_MISO"},
      {0x00008000, "SPI_MOSI"},    {0x00010000, "SPI_CS"},      {0x00020000, "I2C_SDA"},
      {0x00040000, "I2C_SCL"},     {0x00080000, "LEDArray"},
  };
  std::string out;
  for (const auto& f : flags) {
    if (bits & f.bit) {
      if (!out.empty()) {
        out += ' ';
      }
      out += f.name;
    }
  }
  return out.empty() ? "-" : out;
}

int main() {
  VMXPi vmx(true, 50);
  if (!vmx.IsOpen()) {
    std::fprintf(stderr, "VMX is not open (run as root, and only one process may use the VMX at a time)\n");
    return 1;
  }

  std::printf("Channel indexes by type:\n");
  const VMXChannelType types[] = {FlexDIO, AnalogIn, HiCurrDIO, CommDIO};
  for (VMXChannelType t : types) {
    VMXChannelIndex first = 0;
    uint8_t count = vmx.io.GetNumChannelsByType(t, first);
    if (count == 0) {
      std::printf("  %-10s none\n", TypeName(t));
    } else {
      std::printf("  %-10s %u channels: %u - %u\n", TypeName(t), count, first, first + count - 1);
    }
  }

  std::printf("\nEvery channel (index, type, capabilities):\n");
  for (int i = 0; i < 64; ++i) {
    VMXChannelType t;
    VMXChannelCapability caps;
    if (vmx.io.GetChannelCapabilities(static_cast<VMXChannelIndex>(i), t, caps)) {
      std::printf("  %2d  %-10s %s\n", i, TypeName(t), Capabilities(static_cast<uint32_t>(caps)).c_str());
    }
  }
  return 0;
}
