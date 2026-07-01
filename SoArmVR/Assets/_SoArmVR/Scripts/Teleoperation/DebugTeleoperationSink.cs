using UnityEngine;

namespace SoArmVR.Teleoperation
{
    /// <summary>
    /// 通信実装が決まるまでの暫定 Sink。受け取ったサンプルをログ出力するだけ。
    /// </summary>
    public class DebugTeleoperationSink : MonoBehaviour, ITeleoperationSink
    {
        [SerializeField, Tooltip("毎サンプルをログ出力する（高頻度なので注意）")]
        bool _logEverySample;

        [SerializeField, Tooltip("間引きログ: N サンプルに 1 回だけ出力（_logEverySample が無効なとき）")]
        int _logEvery = 30;

        int _count;

        public void OnSessionBegin()
        {
            _count = 0;
            Debug.Log("[Teleoperation] session begin");
        }

        public void Push(in TeleoperationSample sample)
        {
            if (_logEverySample || (_logEvery > 0 && _count % _logEvery == 0))
            {
                Debug.Log($"[Teleoperation] #{sample.id} t={sample.timestampMs} " +
                          $"pos={sample.position} rot={sample.rotation.eulerAngles} grip={sample.gripper:F2}");
            }

            _count++;
        }

        public void OnSessionEnd()
        {
            Debug.Log($"[Teleoperation] session end ({_count} samples)");
        }

        public void PublishReset()
        {
            // Debug sink does not forward to ROS.
        }
    }
}
