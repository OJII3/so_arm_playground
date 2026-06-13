# msg 再生成 (rosettadds-genmsg) 用の dotnet SDK を含む devShell.
# default シェルを汚さないよう, SoArmVR 専用ツールはここに分離する.
{
  mkShell,
  dotnet-sdk_8,
}:
mkShell {
  packages = [ dotnet-sdk_8 ];
  shellHook = ''
    echo "SoArmVR dotnet shell (rosettadds-genmsg 用)"
  '';
}
