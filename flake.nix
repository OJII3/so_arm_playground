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

      mkOfficialGodotMacos =
        pkgs:
        pkgs.stdenvNoCC.mkDerivation {
          pname = "godot";
          version = "4.6.3-stable-official";

          src = pkgs.fetchurl {
            url = "https://github.com/godotengine/godot/releases/download/4.6.3-stable/Godot_v4.6.3-stable_macos.universal.zip";
            hash = "sha256-MGMPPpsR4Qs1wfkLqIFBhdzsQ/rhpINFFZvnVSxkv+g=";
          };

          nativeBuildInputs = [
            pkgs.makeWrapper
            pkgs.unzip
          ];

          dontUnpack = true;

          installPhase = ''
            runHook preInstall

            mkdir -p "$out/Applications" "$out/bin"
            unzip -q "$src" -d "$out/Applications"
            makeWrapper "$out/Applications/Godot.app/Contents/MacOS/Godot" "$out/bin/godot4"

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

          godotPackage = if pkgs.stdenv.isDarwin then mkOfficialGodotMacos pkgs else pkgs.godot_4;

          godotCompat = pkgs.writeShellApplication {
            name = "godot4";
            text = ''
              exec "${godotPackage}/bin/godot4" "$@"
            '';
          };

          godotEditor = pkgs.writeShellApplication {
            name = "godot4-editor";
            text = ''
              exec "${godotCompat}/bin/godot4" --editor "$@"
            '';
          };

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
            godotCompat
            godotEditor
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

          shellHook = ''
            export PYTHONPATH="$PWD/VRTeleop''${PYTHONPATH:+:$PYTHONPATH}"
          ''
          + pkgs.lib.optionalString (metaXrSimulator != null) ''
            export XR_RUNTIME_JSON="${metaXrSimulator}/meta_openxr_simulator.json"
            export META_XRSIM_CONFIG_JSON="${metaXrSimulator}/config/sim_core_configuration.json"
          ''
          + ''
            echo "SO arm playground dev shell"
            echo "  pytest VRTeleop/tests"
            echo "  cd VRTeleop && python -m vrteleop_bridge --config config/default.json --backend dry-run"
            echo "  godot4 --editor VRTeleop/project.godot"
          ''
          + pkgs.lib.optionalString (metaXrSimulator != null) ''
            echo "  Meta XR Simulator runtime: $XR_RUNTIME_JSON"
            echo "  meta-xr-sim-living-room"
          ''
          + ''
            echo ""
            echo "Optional Python packages are installed through uv in a project venv:"
            echo "  uv venv"
            echo "  uv pip install -e 'VRTeleop[sim]'"
            echo "  uv pip install -e 'VRTeleop[real]'"
          '';
        in
        {
          default = pkgs.mkShell {
            packages = commonPackages ++ linuxRuntimePackages;
            inherit shellHook;
          };

          godot = pkgs.mkShell {
            packages = commonPackages ++ linuxRuntimePackages;
            shellHook = shellHook + ''
              echo "Godot is available in this shell as: godot4"
              echo "godot4 uses the official Godot macOS build on Darwin."
            '';
          };
        }
      );

      formatter = forAllSystems (pkgs: pkgs.nixfmt);

      checks = forAllSystems (
        pkgs:
        let
          python = pkgs.python312.withPackages (
            ps: with ps; [
              pytest
            ]
          );
        in
        {
          vrteleop-unit = pkgs.runCommand "vrteleop-unit-tests" { nativeBuildInputs = [ python ]; } ''
            cp -r ${./.} source
            cd source
            export PYTHONPATH="$PWD/VRTeleop"
            pytest VRTeleop/tests -q
            touch $out
          '';
        }
      );
    };
}
