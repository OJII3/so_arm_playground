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

        /// <summary>指定のワールド姿勢（傾き込み）をそのまま基準としてアンカーを設置する。</summary>
        public void Place(Vector3 worldPosition, Quaternion worldRotation)
        {
            transform.SetPositionAndRotation(worldPosition, worldRotation);
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
