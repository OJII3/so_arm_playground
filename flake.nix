{
  description = "SO arm playground monorepo development environment";

  # nix-ros-overlay のバイナリキャッシュ. ROS パッケージのソースビルドを避ける.
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    # ROS 2 用 overlay (Linux のみ). nixpkgs は overlay 側のピンに任せ follows しない
    # (unstable に follows させると overlay のビルドが壊れやすいため).
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/f891b118c8f4ddb2b6f38d6ce1bfe2f8079552ba";
  };

  outputs =
    { nixpkgs, nix-ros-overlay, ... }:
    let
      systems = [
        "aarch64-darwin"
        "x86_64-darwin"
        "aarch64-linux"
        "x86_64-linux"
      ];

      forAllSystems =
        f:
        nixpkgs.lib.genAttrs systems (
          system:
          f (
            import nixpkgs {
              inherit system;
            }
          )
        );

      # hatayama/unity-cli-loop の `uloop` CLI.
      # npm パッケージ `uloop-cli` は esbuild で単一ファイルにバンドル済みで,
      # 実行時依存は Node 組み込みモジュールのみ. tarball を取得して node でラップする.
      mkUloop =
        pkgs:
        pkgs.stdenv.mkDerivation (finalAttrs: {
          pname = "uloop";
          version = "2.1.6";

          src = pkgs.fetchurl {
            url = "https://registry.npmjs.org/uloop-cli/-/uloop-cli-${finalAttrs.version}.tgz";
            hash = "sha512-e3bieo8Ps9lrYRqGqW/90ahrRmUc5+jhpM/m0y3bSOHB0NhZ60yRV6Qzk/iaw5GPbxhTqn3byjHouU+PTNYSYw==";
          };

          nativeBuildInputs = [ pkgs.makeWrapper ];
          dontBuild = true;

          installPhase = ''
            runHook preInstall
            install -Dm644 dist/cli.bundle.cjs $out/lib/uloop/cli.bundle.cjs
            makeWrapper ${pkgs.lib.getExe pkgs.nodejs_22} $out/bin/uloop \
              --add-flags $out/lib/uloop/cli.bundle.cjs
            runHook postInstall
          '';

          meta = {
            description = "CLI tool for Unity Editor communication via Unity CLI Loop";
            homepage = "https://github.com/hatayama/unity-cli-loop";
            license = pkgs.lib.licenses.mit;
            mainProgram = "uloop";
          };
        });
    in
    {
      packages = forAllSystems (pkgs: {
        uloop = mkUloop pkgs;
      });

      devShells = nixpkgs.lib.genAttrs systems (
        system:
        let
          pkgs = import nixpkgs { inherit system; };
          uloop = mkUloop pkgs;
          isLinux = pkgs.stdenv.isLinux;

          commonPackages = [
            pkgs.git
            pkgs.uv
            uloop
          ];

          linuxRuntimePackages = pkgs.lib.optionals isLinux [
            pkgs.libGL
            pkgs.libx11
            pkgs.libxcursor
            pkgs.libxi
            pkgs.libxrandr
          ];

          shellHook = ''
            echo "SO arm playground dev shell"
          '';
        in
        {
          default = pkgs.mkShell {
            packages = commonPackages ++ linuxRuntimePackages;
            inherit shellHook;
          };

          # ROS 2 開発シェル (`nix develop .#ros`). ros2_ws 用.
          # - Linux: nix-ros-overlay による native ROS 環境 + podman.
          # - macOS: nix-ros-overlay は使えないので podman (+ qemu) のみ.
          #          実機/sim は ros2_ws/podman/ のコンテナで動かす.
          ros =
            if isLinux then
              let
                # overlay 自身がテスト/キャッシュ済みの nixpkgs に overlay を適用し
                # full pkgs (mkShell, colcon, rosPackages 等) を得る.
                rosPkgs = import nix-ros-overlay.inputs.nixpkgs {
                  inherit system;
                  overlays = [ nix-ros-overlay.overlays.default ];
                };
                # crayzeewulf/LibSerial は nixpkgs に無いため自作 (ros2_ws/nix/libserial.nix).
                libserial = rosPkgs.callPackage ./ros2_ws/nix/libserial.nix { };
              in
              import ./ros2_ws/nix/shell.nix {
                pkgs = rosPkgs;
                rosDistro = "jazzy";
                extraPkgs = {
                  libserial-dev = libserial;
                };
                extraPaths = [ rosPkgs.podman ];
              }
            else
              pkgs.mkShell {
                packages = [
                  pkgs.podman
                  pkgs.qemu
                ];
                shellHook = ''
                  echo "ros2_ws: macOS では native ROS は不可。podman を使用 (./podman/run.sh)."
                '';
              };
        }
      );

      formatter = forAllSystems (pkgs: pkgs.nixfmt);
    };
}
