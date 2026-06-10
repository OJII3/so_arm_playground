{
  lib,
  stdenv,
  fetchzip,
  autoPatchelfHook,
}:
let
  version = "1.5.0";
  srcs = {
    aarch64-darwin = {
      url = "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${version}/zenoh-plugin-ros2dds-${version}-aarch64-apple-darwin-standalone.zip";
      hash = "sha256-Q4zYAr6xz8OkDlSj4ESaEbmXQpUMrYzkT28MWYOZ2pU=";
    };
    x86_64-darwin = {
      url = "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${version}/zenoh-plugin-ros2dds-${version}-x86_64-apple-darwin-standalone.zip";
      hash = "sha256-9EHxa42CTxLpELfLZFetxi0Ie0fMn3VjpGYEA+mCii8=";
    };
    aarch64-linux = {
      url = "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${version}/zenoh-plugin-ros2dds-${version}-aarch64-unknown-linux-gnu-standalone.zip";
      hash = "sha256-1Cd9wrLcRcdGkbJduZRJmeMslztfLnJMrQQr2S/uQW8=";
    };
    x86_64-linux = {
      url = "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${version}/zenoh-plugin-ros2dds-${version}-x86_64-unknown-linux-gnu-standalone.zip";
      hash = "sha256-RU5v8g/8FOAIg64z6pYM7ZoeGLWCI9XK8gRE9nY/eKw=";
    };
  };
  src = srcs.${stdenv.hostPlatform.system} or (throw "unsupported system: ${stdenv.hostPlatform.system}");
in
stdenv.mkDerivation {
  pname = "zenoh-bridge-ros2dds";
  inherit version;

  src = fetchzip {
    inherit (src) url hash;
    stripRoot = false;
  };

  nativeBuildInputs = lib.optionals stdenv.hostPlatform.isLinux [ autoPatchelfHook ];

  installPhase = ''
    install -Dm755 zenoh-bridge-ros2dds $out/bin/zenoh-bridge-ros2dds
  '';

  meta = {
    description = "Zenoh bridge for ROS 2 DDS — bridges ROS 2 DDS traffic over Zenoh";
    homepage = "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds";
    license = lib.licenses.asl20;
    platforms = builtins.attrNames srcs;
  };
}
