using UnityEngine;
using UnityEngine.InputSystem;

namespace SoArmVR.Teleoperation
{
    /// <summary>
    /// 右コントローラの入力を受けてテレオペレーションを統括する。
    /// グリップを押した瞬間にワールド整列アンカーを設置し、押下中のあいだ
    /// アンカー基準の相対姿勢をサンプリングして <see cref="ITeleoperationSink"/> へ送る。
    /// グリップを離すとアンカーを消去する。
    /// </summary>
    public class TeleoperationSession : MonoBehaviour
    {
        [Header("References")]
        [SerializeField, Tooltip("姿勢ソース（右コントローラの Transform / ワールド姿勢）")]
        Transform _poseSource;

        [SerializeField]
        WorldAlignedAnchor _anchor;

        [SerializeField, Tooltip("ITeleoperationSink を実装した MonoBehaviour")]
        MonoBehaviour _sinkBehaviour;

        [Header("Input")]
        [SerializeField, Tooltip("テレオペ有効（右グリップ押下中。押した瞬間に設置、離すと消去）")]
        InputActionProperty _teleoperateAction;

        [SerializeField, Tooltip("グリッパ開閉量（右トリガー 0..1）")]
        InputActionProperty _gripperAction;

        ITeleoperationSink _sink;
        bool _active;
        int _sampleId;

        void Awake()
        {
            _sink = _sinkBehaviour as ITeleoperationSink;
            if (_sink == null)
                Debug.LogError("[Teleoperation] Sink が未割り当て、または ITeleoperationSink を実装していません", this);
        }

        void OnEnable()
        {
            _teleoperateAction.action?.Enable();
            _gripperAction.action?.Enable();
        }

        void OnDisable()
        {
            _teleoperateAction.action?.Disable();
            _gripperAction.action?.Disable();
            if (_active)
                EndSession();
        }

        void Update()
        {
            bool hold = _teleoperateAction.action != null && _teleoperateAction.action.IsPressed();
            if (hold && !_active)
                BeginSession();
            else if (!hold && _active)
                EndSession();

            if (_active)
                PushSample();
        }

        void BeginSession()
        {
            if (_anchor == null || _poseSource == null)
                return;

            // グリップを押した瞬間の姿勢でアンカーを設置する
            _anchor.Place(_poseSource.position, _poseSource.rotation);
            _active = true;
            _sampleId = 0;
            _sink?.OnSessionBegin();
        }

        void EndSession()
        {
            _active = false;
            _sink?.OnSessionEnd();
            _anchor?.Clear();
        }

        void PushSample()
        {
            _anchor.ToAnchorSpace(_poseSource.position, _poseSource.rotation,
                out Vector3 localPosition, out Quaternion localRotation);

            var sample = new TeleoperationSample
            {
                timestampMs = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                id = _sampleId++,
                position = localPosition,
                rotation = localRotation,
                gripper = _gripperAction.action != null ? _gripperAction.action.ReadValue<float>() : 0f,
            };
            _sink?.Push(in sample);
        }
    }
}
