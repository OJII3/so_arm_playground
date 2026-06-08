namespace SoArmVR.Teleoperation
{
    /// <summary>
    /// テレオペレーションデータの送信先を抽象化するインターフェース。
    /// 通信手段が未確定のため、当面の実装は <see cref="DebugTeleoperationSink"/> のみ。
    /// 将来この実装を差し替えることで外部送信に対応する。
    /// </summary>
    public interface ITeleoperationSink
    {
        /// <summary>テレオペセッション開始時に一度呼ばれる。</summary>
        void OnSessionBegin();

        /// <summary>セッション中、毎サンプル呼ばれる。</summary>
        void Push(in TeleoperationSample sample);

        /// <summary>テレオペセッション終了時に一度呼ばれる。</summary>
        void OnSessionEnd();
    }
}
