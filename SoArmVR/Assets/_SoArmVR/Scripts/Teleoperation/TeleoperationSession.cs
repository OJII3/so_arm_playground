using UnityEngine;
using UnityEngine.InputSystem;

namespace SoArmVR.Teleoperation
{
    /// <summary>
    /// 右コントローラの入力を受けてテレオペレーションを統括する。
    /// A ボタンでワールド整列アンカーを設置し、グリップ押下中のあいだ
    /// アンカー基準の相対姿勢をサンプリングして <see cref="ITeleoperationSink"/> へ送る。
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
        [SerializeField, Tooltip("アンカー設置（右 A ボタン）")]
        InputActionProperty _placeAnchorAction;

        [SerializeField, Tooltip("テレオペ有効（右グリップ押下中）")]
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
            _placeAnchorAction.action?.Enable();
            _teleoperateAction.action?.Enable();
            _gripperAction.action?.Enable();
        }

        void OnDisable()
        {
            _placeAnchorAction.action?.Disable();
            _teleoperateAction.action?.Disable();
            _gripperAction.action?.Disable();
            if (_active)
                EndSession();
        }

        void Update()
        {
            if (_placeAnchorAction.action != null && _placeAnchorAction.action.WasPressedThisFrame())
                PlaceAnchor();

            bool hold = _teleoperateAction.action != null && _teleoperateAction.action.IsPressed();
            if (hold && !_active)
                BeginSession();
            else if (!hold && _active)
                EndSession();

            if (_active && _poseSource != null && _anchor != null && _anchor.IsPlaced)
                PushSample();
        }

        void PlaceAnchor()
        {
            if (_anchor == null || _poseSource == null)
                return;

            _anchor.Place(_poseSource.position, _poseSource.rotation);
        }

        void BeginSession()
        {
            // アンカー未設置のあいだは開始しない（押し続けていれば設置後に開始される）
            if (_anchor == null || !_anchor.IsPlaced)
                return;

            _active = true;
            _sampleId = 0;
            _sink?.OnSessionBegin();
        }

        void EndSession()
        {
            _active = false;
            _sink?.OnSessionEnd();
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
