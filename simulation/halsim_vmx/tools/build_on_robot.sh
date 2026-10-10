#!/usr/bin/env bash
# build_on_robot.sh - build libhalsim_vmx.so and the test tools ON the VMX-pi.
#
#   ./simulation/halsim_vmx/tools/build_on_robot.sh [--skip-download] [--dry-run]
#
# Run it from an allwpilib checkout on the robot (it needs g++ >= 11, python3 and curl, and
# VMXPi.h for vmx_channels). It does not build WPILib: it downloads the published linuxarm64
# libraries (about 42 MB) from frcmaven.wpi.edu, and links the extension against them.
#
# Outputs (default ~/halsim_vmx-build):
#   libhalsim_vmx.so   the HALSIM extension (HALSIM_EXTENSIONS)
#   hal_dio_test       drives WPILib DIO through the sim HAL
#   vmx_channels       prints the VMX channel map
# Libraries (default ~/wpilib-libs): libwpiHal.so, libwpiutil.so, libntcore.so, libdatalog.so, libwpinet.so
#
# NOT YET RUN on a robot.

set -euo pipefail

VERSION="${WPILIB_VERSION:-2027.0.0-alpha-7}"
MAVEN="https://frcmaven.wpi.edu/artifactory/release/org/wpilib"
LIBS_DIR="${LIBS_DIR:-$HOME/wpilib-libs}"
OUT_DIR="${OUT_DIR:-$HOME/halsim_vmx-build}"
SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AW_DIR="$(cd "$SELF_DIR/../../.." && pwd)"          # allwpilib root: simulation/halsim_vmx/tools -> ../../..
EXT_DIR="$AW_DIR/simulation/halsim_vmx"
DRY_RUN=0
SKIP_DOWNLOAD=0

# group/artifact of the libraries libwpiHal.so needs (see `readelf -d libwpiHal.so`)
ARTIFACTS=(hal/hal-cpp wpiutil/wpiutil-cpp ntcore/ntcore-cpp datalog/datalog-cpp wpinet/wpinet-cpp)

die() { echo "build_on_robot.sh: $*" >&2; exit 1; }
info() { echo "==> $*"; }

for arg in "$@"; do
  case "$arg" in
    --skip-download) SKIP_DOWNLOAD=1 ;;
    --dry-run) DRY_RUN=1 ;;
    -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
    *) die "unknown argument: $arg" ;;
  esac
done

run() {
  if [ "$DRY_RUN" = 1 ]; then printf '[dry-run]'; printf ' %q' "$@"; printf '\n'; else "$@"; fi
}

[ -d "$EXT_DIR/src/main/native/cpp" ] || die "halsim_vmx sources not found at $EXT_DIR (run from an allwpilib checkout)"
[ -d "$AW_DIR/hal/src/main/native/include" ] || die "HAL headers not found under $AW_DIR/hal"
[ -d "$AW_DIR/wpiutil/src/main/native/include" ] || die "wpiutil headers not found under $AW_DIR/wpiutil"
command -v g++ >/dev/null 2>&1 || die "g++ is required (sudo apt install -y g++)"
command -v python3 >/dev/null 2>&1 || die "python3 is required"
gpp_major="$(g++ -dumpversion | cut -d. -f1)"
[ "$gpp_major" -ge 11 ] || die "g++ >= 11 is required (C++20), found $gpp_major"
if [ "$SKIP_DOWNLOAD" != 1 ]; then command -v curl >/dev/null 2>&1 || die "curl is required (or pass --skip-download)"; fi

# ---------------------------------------------------------------- libraries
info "WPILib $VERSION linuxarm64 libraries -> $LIBS_DIR"
run mkdir -p "$LIBS_DIR"
for ga in "${ARTIFACTS[@]}"; do
  group="${ga%/*}"; name="${ga#*/}"
  marker="$LIBS_DIR/.$name-$VERSION"
  if [ -f "$marker" ]; then echo "  $name: already extracted"; continue; fi
  if [ "$SKIP_DOWNLOAD" = 1 ]; then echo "  $name: skipped (--skip-download)"; continue; fi
  url="$MAVEN/$group/$name/$VERSION/$name-$VERSION-linuxarm64.zip"
  zip="$LIBS_DIR/$name-$VERSION.zip"
  echo "  $name: downloading $url"
  run curl -fL -sS -o "$zip" "$url"
  if [ "$DRY_RUN" = 1 ]; then continue; fi
  # shared libraries only (not the .debug files)
  python3 - "$zip" "$LIBS_DIR" <<'PY'
import os, sys, zipfile
zip_path, out = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(zip_path) as z:
    for info in z.infolist():
        base = os.path.basename(info.filename)
        if "/shared/" in info.filename and ".so" in base and not base.endswith(".debug"):
            with z.open(info) as src, open(os.path.join(out, base), "wb") as dst:
                dst.write(src.read())
            print("    extracted", base)
PY
  rm -f "$zip"
  touch "$marker"
done

# ---------------------------------------------------------------- build
INC=(-I"$AW_DIR/hal/src/main/native/include" -I"$AW_DIR/wpiutil/src/main/native/include")
LINK=(-L"$LIBS_DIR" "-Wl,-rpath,$LIBS_DIR" -lwpiHal -lwpiutil -lntcore -ldatalog -lwpinet)
run mkdir -p "$OUT_DIR"

info "libhalsim_vmx.so"
run g++ -std=c++20 -shared -fPIC -O2 -Wall -Wextra "${INC[@]}" -I"$EXT_DIR/src/main/native/include" \
  "$EXT_DIR"/src/main/native/cpp/{BackendFactory,ChannelMap,DlBackend,HALSimVMX,LoopbackBackend,main}.cpp \
  "${LINK[@]}" -ldl -lpthread -o "$OUT_DIR/libhalsim_vmx.so"

info "hal_dio_test"
run g++ -std=c++20 -O2 -Wall -Wextra "${INC[@]}" "$EXT_DIR/tools/hal_dio_test.cpp" "${LINK[@]}" -lpthread -o "$OUT_DIR/hal_dio_test"

info "vmx_channels"
if [ -f /usr/local/include/vmxpi/VMXPi.h ]; then
  run g++ -std=c++17 -O2 -Wall -Wextra -I/usr/local/include/vmxpi "$EXT_DIR/tools/vmx_channels.cpp" \
    -L/usr/local/lib/vmxpi -lvmxpi_hal_cpp -lrt -lpthread -latomic -o "$OUT_DIR/vmx_channels"
else
  echo "  skipped: /usr/local/include/vmxpi/VMXPi.h not found"
fi

# ---------------------------------------------------------------- check the libraries load
if [ "$DRY_RUN" != 1 ]; then
  info "ldd check (anything printed below is a problem)"
  for f in "$OUT_DIR/libhalsim_vmx.so" "$OUT_DIR/hal_dio_test"; do
    LD_LIBRARY_PATH="$LIBS_DIR" ldd "$f" | grep "not found" | sed "s|^|  $(basename "$f"): |" || true
  done
  ldconfig -p | grep -q 'libatomic.so.1' || echo "  libatomic.so.1 is missing: sudo apt install -y libatomic1"
fi

cat <<EOF

==> Done$( [ "$DRY_RUN" = 1 ] && echo " (dry run: nothing was built)" )
Find the VMX channel numbers first (the VMX must not be in use by another process):
  sudo $OUT_DIR/vmx_channels

Then drive a DIO output through the simulation HAL (WPILib DIO 0 -> VMX channel <N>; DIO 1 -> VMX channel <M>, wired to <N> for a loopback):
  sudo env LD_LIBRARY_PATH=$LIBS_DIR \\
    LD_PRELOAD=/opt/robot_manager/lib/libvmx_gpio_isr_shim.so \\
    HALSIM_EXTENSIONS=$OUT_DIR/libhalsim_vmx.so \\
    HALSIMVMX_BACKEND=/opt/halsim_vmx/libhalsim_vmx_studica.so \\
    HALSIMVMX_DIO_MAP="0:<N>,1:<M>" \\
    $OUT_DIR/hal_dio_test 0 1
EOF
