# crayzeewulf/LibSerial — nixpkgs に存在しないため自作する。
# feetech_ros2_driver の feetech_driver サブパッケージが
# `pkg_check_modules(SERIAL libserial REQUIRED)` で要求するため、
# pkg-config 用の libserial.pc を含むこのライブラリを提供する。
#
# v1.0.0 タグは tests/docs/python(sip) を無条件にビルドし重い依存を要求するため、
# それらを無効化できる option を備えた master の固定コミットを使う。
{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  pkg-config,
}:
stdenv.mkDerivation {
  pname = "libserial";
  version = "1.0.0-unstable-2025-09-03";

  src = fetchFromGitHub {
    owner = "crayzeewulf";
    repo = "libserial";
    rev = "50e0f443666d48d7c7e181dc73a6b35700517fae";
    sha256 = "0jsb6ad0psgx149zs9g8dc665qmpdlp83nrh2fvmk3vfgvfz514b";
  };

  nativeBuildInputs = [
    cmake
    pkg-config
  ];

  cmakeFlags = [
    "-DLIBSERIAL_ENABLE_TESTING=OFF"
    "-DLIBSERIAL_BUILD_EXAMPLES=OFF"
    "-DLIBSERIAL_PYTHON_ENABLE=OFF"
    "-DLIBSERIAL_BUILD_DOCS=OFF"
  ];

  meta = {
    description = "C++ library for serial port programming on POSIX systems";
    homepage = "https://github.com/crayzeewulf/libserial";
    license = lib.licenses.bsd3;
    platforms = lib.platforms.linux;
  };
}
