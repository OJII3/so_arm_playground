final: prev:
{
  feetech-ros2-driver = final.callPackage ./feetech-ros2-driver.nix {};
  lerobot-controller = final.callPackage ./lerobot-controller.nix {};
  lerobot-description = final.callPackage ./lerobot-description.nix {};
  lerobot-moveit = final.callPackage ./lerobot-moveit.nix {};
}
