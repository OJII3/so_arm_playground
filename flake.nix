{
  description = "SO arm playground monorepo development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    { nixpkgs, ... }:
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

      mkMetaXrSimulator =
        pkgs:
        pkgs.stdenvNoCC.mkDerivation {
          pname = "meta-xr-simulator";
          version = "71.0.0";

          src = pkgs.fetchurl {
            url = "https://media.githubusercontent.com/media/Oculus-VR/homebrew-repo/main/repo/meta-xr-simulator/meta-xr-simulator-71.0.0.tar.gz";
            hash = "sha256-19CsNA/RtFdfGX+911Yx4YBfv0K4Vghvioass6Fr19Y=";
          };

          sourceRoot = "meta-xr-simulator-71.0.0-beta.3";

          installPhase = ''
            runHook preInstall

            mkdir -p "$out"
            cp -R . "$out/"
            chmod +x "$out"/synth_env_server/*.sh

            runHook postInstall
          '';
        };
    in
    {
      packages = forAllSystems (
        pkgs:
        pkgs.lib.optionalAttrs (pkgs.stdenv.hostPlatform.system == "aarch64-darwin") rec {
          meta-xr-simulator = mkMetaXrSimulator pkgs;
          default = meta-xr-simulator;
        }
      );

      devShells = forAllSystems (
        pkgs:
        let
          metaXrSimulator =
            if pkgs.stdenv.hostPlatform.system == "aarch64-darwin" then mkMetaXrSimulator pkgs else null;

          python = pkgs.python312.withPackages (
            ps: with ps; [
              pytest
            ]
          );

          metaXrLivingRoom = pkgs.writeShellApplication {
            name = "meta-xr-sim-living-room";
            text = ''
              exec "${metaXrSimulator}/synth_env_server/LaunchLivingRoom.sh"
            '';
          };

          metaXrGameRoom = pkgs.writeShellApplication {
            name = "meta-xr-sim-game-room";
            text = ''
              exec "${metaXrSimulator}/synth_env_server/LaunchGameRoom.sh"
            '';
          };

          metaXrBedroom = pkgs.writeShellApplication {
            name = "meta-xr-sim-bedroom";
            text = ''
              exec "${metaXrSimulator}/synth_env_server/LaunchBedroom.sh"
            '';
          };

          metaXrPackages = pkgs.lib.optionals (metaXrSimulator != null) [
            metaXrSimulator
            metaXrLivingRoom
            metaXrGameRoom
            metaXrBedroom
          ];

          commonPackages = [
            python
            pkgs.git
            pkgs.uv
          ]
          ++ metaXrPackages;

          linuxRuntimePackages = pkgs.lib.optionals pkgs.stdenv.isLinux [
            pkgs.libGL
            pkgs.xorg.libX11
            pkgs.xorg.libXcursor
            pkgs.xorg.libXi
            pkgs.xorg.libXrandr
          ];

          shellHook =
            pkgs.lib.optionalString (metaXrSimulator != null) ''
              export XR_RUNTIME_JSON="${metaXrSimulator}/meta_openxr_simulator.json"
              export META_XRSIM_CONFIG_JSON="${metaXrSimulator}/config/sim_core_configuration.json"
            ''
            + ''
              echo "SO arm playground dev shell"
            ''
            + pkgs.lib.optionalString (metaXrSimulator != null) ''
              echo "  Meta XR Simulator runtime: $XR_RUNTIME_JSON"
              echo "  meta-xr-sim-living-room"
            '';
        in
        {
          default = pkgs.mkShell {
            packages = commonPackages ++ linuxRuntimePackages;
            inherit shellHook;
          };
        }
      );

      formatter = forAllSystems (pkgs: pkgs.nixfmt);
    };
}
