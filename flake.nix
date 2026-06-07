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

      devShells = forAllSystems (
        pkgs:
        let
          uloop = mkUloop pkgs;

          commonPackages = [
            pkgs.git
            pkgs.uv
            uloop
          ];

          linuxRuntimePackages = pkgs.lib.optionals pkgs.stdenv.isLinux [
            pkgs.libGL
            pkgs.xorg.libX11
            pkgs.xorg.libXcursor
            pkgs.xorg.libXi
            pkgs.xorg.libXrandr
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
      );

      formatter = forAllSystems (pkgs: pkgs.nixfmt);
    };
}
