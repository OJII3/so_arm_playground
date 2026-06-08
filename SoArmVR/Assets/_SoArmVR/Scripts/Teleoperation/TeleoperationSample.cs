using System;
using UnityEngine;

namespace SoArmVR.Teleoperation
{
    /// <summary>
    /// アンカー基準で記録したテレオペレーションの 1 サンプル。
    /// のちほど <see cref="ITeleoperationSink"/> 経由で外部送信する想定。
    /// 通信仕様が未確定のため、フィールド構成は暫定。
    /// </summary>
    [Serializable]
    public struct TeleoperationSample
    {
        /// <summary>Unix エポックからの経過ミリ秒。</summary>
        public long timestampMs;

        /// <summary>サンプル連番（暫定の固有 ID）。</summary>
        public int id;

        /// <summary>アンカー基準の相対位置。</summary>
        public Vector3 position;

        /// <summary>アンカー基準の相対回転。</summary>
        public Quaternion rotation;

        /// <summary>グリッパ開閉量（0..1, 右トリガー）。</summary>
        public float gripper;
    }
}
