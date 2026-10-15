#!/usr/bin/env python3
"""Cross-build VMX runtime libraries using an SDK already fetched from the user's robot."""
import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', type=Path, required=True)
    parser.add_argument('--sdk', type=Path, required=True, help='SDK root containing usr/local/include/vmxpi and usr/local/lib/vmxpi')
    parser.add_argument('--wpilib-libs', type=Path, required=True, help='Directory containing matching ARM64 WPILib shared libraries')
    parser.add_argument('--output', type=Path, required=True, help='Output vmx folder (include/, linuxarm64/, licenses/)')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    extension = root / 'simulation/halsim_vmx'
    drivers = root / 'studica_drivers'
    sdk_headers = args.sdk / 'usr/local/include/vmxpi'
    sdk_libs = args.sdk / 'usr/local/lib/vmxpi'
    if not (sdk_headers / 'VMXPi.h').is_file() or not (sdk_libs / 'libvmxpi_hal_cpp.so').is_file():
        parser.error('Incomplete robot VMX SDK')
    for name in ['libwpiHal.so', 'libwpiutil.so', 'libwpiHald.so', 'libwpiutild.so']:
        if not (args.wpilib_libs / name).is_file():
            parser.error('Missing WPILib ARM64 library: ' + name)
    args.output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='vmx-runtime-', dir=args.output.parent) as temporary:
        staging = Path(temporary)
        library = staging / 'linuxarm64'
        library.mkdir()
        common = [str(args.compiler), '-std=c++20', '-O2', '-fPIC', '-shared', '-pthread', '-Wl,--no-undefined',
                  '-I' + str(sdk_headers), '-I' + str(drivers), '-L' + str(sdk_libs),
                  '-Wl,-rpath,/usr/local/lib/vmxpi', '-Wl,-rpath,$ORIGIN']

        def compile_library(name, sources, flags):
            print('Building ' + name, flush=True)
            subprocess.run(common + [str(source) for source in sources] + flags + ['-Wl,-soname,' + name, '-o', str(library / name)], check=True)

        compile_library('libstudica_drivers.so', sorted(drivers.glob('*.cpp')), ['-lvmxpi_hal_cpp', '-latomic', '-lrt'])
        compile_library('libhalsim_vmx_studica.so', sorted((extension / 'src/studica/native/cpp').glob('*.cpp')),
                        ['-I' + str(extension / 'src/main/native/include'), '-I' + str(extension / 'src/studica/native/include'),
                         '-L' + str(library), '-lstudica_drivers', '-lvmxpi_hal_cpp'])
        compile_library('libhalsim_vmx.so', sorted((extension / 'src/main/native/cpp').glob('*.cpp')),
                        ['-I' + str(extension / 'src/main/native/include'), '-I' + str(root / 'hal/src/main/native/include'),
                         '-I' + str(root / 'wpiutil/src/main/native/include'), '-L' + str(args.wpilib_libs),
                         '-Wl,-rpath-link,' + str(args.wpilib_libs), '-lwpiHal', '-lwpiutil', '-ldl'])
        # Debug WPILib has distinct SONAMEs. Reusing the release extension would
        # load a second HAL and disconnect the hardware from the robot's HAL state.
        compile_library('libhalsim_vmxd.so', sorted((extension / 'src/main/native/cpp').glob('*.cpp')),
                        ['-O0', '-g', '-I' + str(extension / 'src/main/native/include'), '-I' + str(root / 'hal/src/main/native/include'),
                         '-I' + str(root / 'wpiutil/src/main/native/include'), '-L' + str(args.wpilib_libs),
                         '-Wl,-rpath-link,' + str(args.wpilib_libs), '-lwpiHald', '-lwpiutild', '-ldl'])
        headers = staging / 'include'
        headers.mkdir()
        for source in drivers.glob('*.hpp'):
            shutil.copy2(source, headers / source.name)
        for source in [extension / 'src/main/native/include', extension / 'src/studica/native/include']:
            shutil.copytree(source, headers, dirs_exist_ok=True)
        notices = staging / 'licenses'
        notices.mkdir()
        for source, name in [(root / 'LICENSE.md', 'WPILib-LICENSE.md'), (root / 'ThirdPartyNotices.txt', 'WPILib-ThirdPartyNotices.txt'),
                             (drivers / 'LICENSE', 'Studica-LICENSE'), (drivers / 'SOURCE.txt', 'Studica-SOURCE.txt')]:
            shutil.copy2(source, notices / name)
        # SDK headers and proprietary/image-specific HAL libraries are intentionally not copied.
        shutil.copytree(staging, args.output, dirs_exist_ok=True)
    print('VMX runtime and public headers: ' + str(args.output))


if __name__ == '__main__':
    main()
