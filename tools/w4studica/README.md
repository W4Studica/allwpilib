# W4Studica release builds

The WPILib baseline stays 2027.0.0-alpha-7. W4Studica distributions use a distinct fixed Maven version, `2027.0.0-alpha-7-w4.beta.N`, published as GitHub pre-releases (tag `v<version>`).

## 1. Maven repository

With JDK 25 and the native toolchains installed, run one build per native platform into the same staging repository (Java artifacts are platform-independent):

```sh
V=2027.0.0-alpha-7-w4.beta.N
for p in linuxarm64 linuxx86-64 linuxsystemcore; do
  ./gradlew publishToMavenLocal -PpublishVersion=$V -Ponly$p -PskipJavaFormat \
    -Dmaven.repo.local=/path/to/staging-$p/maven
done
```

Merge the three repositories; identical files are kept once, so the module metadata keeps every platform's classifiers.

## 2. VMX runtime

Build the VMX libraries with an SDK already fetched from the user's own robot (`fetchVmxSdk<target>`):

```sh
python3 tools/w4studica/build_vmx_runtime.py \
  --compiler /path/to/aarch64-trixie-linux-gnu-g++ \
  --sdk /path/to/vmxsdk \
  --wpilib-libs /path/to/extracted/linuxarm64/shared \
  --output /path/to/staging/vmx
```

The ARM64 library directory must hold the release and debug WPILib libraries. The script builds `libhalsim_vmx.so` and `libhalsim_vmxd.so` (they match the release and debug HAL SONAMEs), `libhalsim_vmx_studica.so` and `libstudica_drivers.so`. The debug and release extensions share the backend and drivers, which do not depend on the WPILib HAL. `studica_drivers` is left unchanged. Only the new libraries, the public headers and the license notices are packaged: **`VMXPi.h`, the VMX HAL and the SDK are never redistributed.**

## 3. Package and publish

Use `tools/package_release.py` in the VS Code extension repository to make the allwpilib ZIPs (split at 1.4 GB) and the `w4studica.json` manifest. The release notes must list the platform classifiers actually included, the required runtime versions, the source commit and the tests performed. Do not claim Windows, macOS or other native binaries that were not built.
