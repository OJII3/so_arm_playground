using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using UnityEngine;

namespace SoArmVR.Teleoperation
{
    /// <summary>
    /// ローカル NIC の IPv4 アドレスを解決するヘルパー。
    ///
    /// 本来 ROSettaDDS (DDS/rmw 層) が全 NIC を自動列挙してユニキャスト locator を
    /// 広告すべきだが、現状は既定で loopback を広告するため、LAN 上の ROS 2 と
    /// 通信するには実 NIC の IP を明示する必要がある。
    /// これはその暫定対応であり、ROSettaDDS がインターフェース自動列挙に対応すれば不要になる。
    /// 参照: docs/rosettadds-feature-gaps.md
    /// </summary>
    public static class LocalNetwork
    {
        /// <summary>
        /// up・非 loopback・IPv4 の最初の有効なユニキャストアドレスを返す。
        /// 見つからなければ null。
        /// </summary>
        public static IPAddress ResolvePrimaryIPv4()
        {
            foreach (var nic in NetworkInterface.GetAllNetworkInterfaces())
            {
                if (nic.OperationalStatus != OperationalStatus.Up) continue;
                if (nic.NetworkInterfaceType == NetworkInterfaceType.Loopback) continue;

                foreach (var ua in nic.GetIPProperties().UnicastAddresses)
                {
                    if (ua.Address.AddressFamily != AddressFamily.InterNetwork) continue;
                    if (IPAddress.IsLoopback(ua.Address)) continue;
                    return ua.Address;
                }
            }

            Debug.LogWarning(
                "LocalNetwork: 有効な IPv4 NIC が見つかりませんでした。loopback にフォールバックします。");
            return null;
        }
    }
}
