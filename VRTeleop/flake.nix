{
  description = "Development environment for VRTeleop";

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
    in
    {
      devShells = forAllSystems (
        pkgs:
        let
          python = pkgs.python312.withPackages (
            ps: with ps; [
              pytest
            ]
          );

          commonPackages = [
            python
            pkgs.git
            pkgs.uv
          ];

          linuxRuntimePackages = pkgs.lib.optionals pkgs.stdenv.isLinux [
            pkgs.libGL
            pkgs.xorg.libX11
            pkgs.xorg.libXcursor
            pkgs.xorg.libXi
            pkgs.xorg.libXrandr
          ];

          shellHook = ''
            export PYTHONPATH="$PWD''${PYTHONPATH:+:$PYTHONPATH}"
            echo "VRTeleop dev shell"
            echo "  python -m vrteleop_bridge --config config/default.json --backend dry-run"
            echo "  pytest -q"
            echo ""
            echo "MuJoCo and LeRobot are installed through Python packaging, not nixpkgs:"
            echo "  uv venv"
            echo "  uv pip install -e '.[sim]'"
            echo "  pip install 'lerobot[feetech]'"
          '';
        in
        {
          default = pkgs.mkShell {
            packages = commonPackages ++ linuxRuntimePackages;
            inherit shellHook;
          };

          godot = pkgs.mkShell {
            packages =
              commonPackages
              ++ linuxRuntimePackages
              ++ [
                pkgs.godot_4
              ];
            shellHook = shellHook + ''
              echo "Godot is available in this shell as: godot4"
              echo "For Quest/OpenXR work on macOS, a user-installed Godot.app is usually easier."
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
          unit = pkgs.runCommand "vrteleop-unit-tests" { nativeBuildInputs = [ python ]; } ''
            cp -r ${./.} source
            cd source
            export PYTHONPATH="$PWD"
            pytest -q
            touch $out
          '';
        }
      );
    };
}
