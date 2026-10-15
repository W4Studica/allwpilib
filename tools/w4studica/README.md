# W4Studica beta release builds

The WPILib baseline remains 2027.0.0-alpha-7. W4Studica distributions use a distinct
fixed Maven version such as `2027.0.0-alpha-7-w4.beta.1` and GitHub pre-releases.

Build the Maven repository from the checkout with JDK 25 and the appropriate
native toolchain installed. Run separate native-platform builds into the same
staging repository; Java artifacts are platform-independent.

```sh
./gradlew publishToMavenLocal -PpublishVersion=2027.0.0-alpha-7-w4.beta.1 \
  -Ponlylinuxarm64 -PskipJavaFormat -Dmaven.repo.local=/path/to/staging/maven
./gradlew publishToMavenLocal -PpublishVersion=2027.0.0-alpha-7-w4.beta.1 \
  -Ponlylinuxx86-64 -PskipJavaFormat -Dmaven.repo.local=/path/to/staging/maven
```

Build VMX runtime libraries using an SDK already fetched from the user's robot:

```sh
python3 tools/w4studica/build_vmx_runtime.py \
  --compiler /path/to/aarch64-trixie-linux-gnu-g++ \
  --sdk /path/to/vmxsdk \
  --wpilib-libs /path/to/extracted/linuxarm64/shared \
  --output /path/to/staging/vmx
```

The ARM64 library directory must contain release and debug WPILib libraries.
The runtime builder creates separate `libhalsim_vmx.so` and `libhalsim_vmxd.so`
extensions matching the release/debug HAL SONAMEs. Both variants share the same
Studica backend and driver library, which do not depend on the WPILib HAL.

The runtime builder leaves `studica_drivers` unchanged. It packages only the
newly built extension, backend, Studica drivers, public project headers and
license notices. It does not redistribute `VMXPi.h`, the VMX HAL or the SDK.

Use the VS Code extension repository's `tools/package_release.py` to produce the
allwpilib Maven ZIP and `w4studica.json` manifest. Release notes must list the
platform classifiers actually included, required runtime versions, source
commit and the tests performed. Do not claim unbuilt Windows/macOS/SystemCore
native binaries are supported by a Linux/VMX-only beta.
