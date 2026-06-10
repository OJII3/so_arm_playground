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
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/f42aabf01c2c1d5754021549c26591b2d422ac10";
  };

  outputs =
    { nixpkgs, nix-ros-overlay, ... }:
    let
      systems = [
        "aarch64-darwin"
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

      mkUloop = pkgs: pkgs.callPackage ./SoArmVR/nix/uloop.nix { };
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
            pkgs.podman
            pkgs.socat
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
        }
        # ROS 2 開発シェル (`nix develop .#ros`). nix-ros-overlay は Linux のみ実用なため
        # darwin では定義しない (その場合は ros2_ws/podman/ を使う).
        // pkgs.lib.optionalAttrs isLinux {
          ros =
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
            };
        }
      );

      formatter = forAllSystems (pkgs: pkgs.nixfmt);
    };
}
