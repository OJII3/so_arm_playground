// teleop_ik/test/test_packaging.cpp
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace fs = std::filesystem;

TEST(Packaging, InstallLibDirectoryContainsExecutables)
{
  const auto share = ament_index_cpp::get_package_share_directory("teleop_ik");
  // share ディレクトリ: <prefix>/share/teleop_ik
  // → prefix = share.parent_path().parent_path()
  // 実行ファイル: <prefix>/lib/teleop_ik/
  const fs::path prefix = fs::path(share).parent_path().parent_path();
  const fs::path lib_dir = prefix / "lib" / "teleop_ik";
  EXPECT_TRUE(fs::exists(lib_dir / "teleop_ik_node"));
  EXPECT_TRUE(fs::exists(lib_dir / "gamepad_teleop_node"));
}
