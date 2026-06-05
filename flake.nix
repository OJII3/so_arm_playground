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

          godotEditor = pkgs.writeShellApplication {
            name = "godot4-editor";
            runtimeInputs = [ pkgs.godot_4 ];
            text = ''
              exec godot4 --rendering-driver opengl3 --rendering-method gl_compatibility "$@"
            '';
          };

          commonPackages = [
            python
            pkgs.git
            pkgs.godot_4
            godotEditor
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
            export PYTHONPATH="$PWD/VRTeleop''${PYTHONPATH:+:$PYTHONPATH}"
            echo "SO arm playground dev shell"
            echo "  pytest VRTeleop/tests"
            echo "  cd VRTeleop && python -m vrteleop_bridge --config config/default.json --backend dry-run"
            echo "  godot4-editor --editor VRTeleop/project.godot"
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
              echo "Use godot4-editor for the editor on macOS if the Metal renderer crashes."
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
