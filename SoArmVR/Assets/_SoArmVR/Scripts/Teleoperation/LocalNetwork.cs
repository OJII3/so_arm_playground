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
                    // 列挙順で最初に見つかった NIC を採用するため、VPN や WiFi Direct の
                    // 仮想 NIC が物理 LAN より先に来ると誤検出しうる。採用した IP をログして
                    // 疎通不能時に切り分けられるようにする。
                    Debug.Log($"LocalNetwork: 使用する IPv4 = {ua.Address} (NIC: {nic.Name})");
                    return ua.Address;
                }
            }

            Debug.LogWarning(
                "LocalNetwork: 有効な IPv4 NIC が見つかりませんでした。loopback にフォールバックします。");
            return null;
        }
    }
}
