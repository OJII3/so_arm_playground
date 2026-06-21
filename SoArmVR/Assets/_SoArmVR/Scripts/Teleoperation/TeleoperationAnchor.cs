using UnityEngine;

namespace SoArmVR.Teleoperation
{
    /// <summary>
    /// テレオペレーションの基準フレーム。設置した瞬間のコントローラ姿勢
    /// （位置・回転とも全軸、傾き込み）をそのまま基準とし、
    /// ワールド姿勢をアンカー基準の相対姿勢へ変換する機能を提供する。
    /// </summary>
    public class TeleoperationAnchor : MonoBehaviour
    {
        [SerializeField, Tooltip("設置前は隠しておく見た目（任意）")]
        GameObject _visual;

        /// <summary>アンカーが設置済みか。</summary>
        public bool IsPlaced { get; private set; }

        void Awake()
        {
            SetVisualVisible(false);
        }

        /// <summary>
        /// 指定のワールド位置とワールド回転を基準にアンカーを設置する。
        /// アンカーの up 軸は常にワールド up 軸に一致させ、ヨーのみを保持する
        /// (コントローラの pitch/roll は捨てる)。
        /// </summary>
        public void Place(Vector3 worldPosition, Quaternion worldRotation)
        {
            Vector3 fwd = Vector3.ProjectOnPlane(worldRotation * Vector3.forward, Vector3.up);
            if (fwd.sqrMagnitude < 1e-6f)
            {
                // 真上/真下を向きすぎて水平成分がほぼゼロのときはワールド forward にフォールバック
                fwd = Vector3.ProjectOnPlane(Vector3.forward, Vector3.up);
            }
            transform.SetPositionAndRotation(worldPosition, Quaternion.LookRotation(fwd.normalized, Vector3.up));
            IsPlaced = true;
            SetVisualVisible(true);
        }

        /// <summary>アンカーを未設置状態に戻し、見た目を隠す。</summary>
        public void Clear()
        {
            IsPlaced = false;
            SetVisualVisible(false);
        }

        /// <summary>ワールド姿勢をアンカー基準の相対姿勢へ変換する。</summary>
        public void ToAnchorSpace(Vector3 worldPosition, Quaternion worldRotation,
            out Vector3 localPosition, out Quaternion localRotation)
        {
            localPosition = transform.InverseTransformPoint(worldPosition);
            localRotation = Quaternion.Inverse(transform.rotation) * worldRotation;
        }

        void SetVisualVisible(bool visible)
        {
            if (_visual != null)
                _visual.SetActive(visible);
        }
    }
}
