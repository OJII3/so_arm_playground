# hatayama/unity-cli-loop の `uloop` CLI.
# npm パッケージ `uloop-cli` は esbuild で単一ファイルにバンドル済みで,
# 実行時依存は Node 組み込みモジュールのみ. tarball を取得して node でラップする.
{
  lib,
  stdenv,
  fetchurl,
  makeWrapper,
  nodejs_22,
}:
stdenv.mkDerivation (finalAttrs: {
  pname = "uloop";
  version = "2.1.6";

  src = fetchurl {
    url = "https://registry.npmjs.org/uloop-cli/-/uloop-cli-${finalAttrs.version}.tgz";
    hash = "sha512-e3bieo8Ps9lrYRqGqW/90ahrRmUc5+jhpM/m0y3bSOHB0NhZ60yRV6Qzk/iaw5GPbxhTqn3byjHouU+PTNYSYw==";
  };

  nativeBuildInputs = [ makeWrapper ];
  dontBuild = true;

  installPhase = ''
    runHook preInstall
    install -Dm644 dist/cli.bundle.cjs $out/lib/uloop/cli.bundle.cjs
    makeWrapper ${lib.getExe nodejs_22} $out/bin/uloop \
      --add-flags $out/lib/uloop/cli.bundle.cjs
    runHook postInstall
  '';

  meta = {
    description = "CLI tool for Unity Editor communication via Unity CLI Loop";
    homepage = "https://github.com/hatayama/unity-cli-loop";
    license = lib.licenses.mit;
    mainProgram = "uloop";
  };
})
