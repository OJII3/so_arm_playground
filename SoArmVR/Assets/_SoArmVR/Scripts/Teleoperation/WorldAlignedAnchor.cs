using UnityEngine;

namespace SoArmVR.Teleoperation
{
    /// <summary>
    /// ワールドの鉛直方向（up）に常に整列するアンカー。
    /// 設置時のコントローラ向きの水平成分（ヨー）を正面とし、
    /// ワールド姿勢をアンカー基準の相対姿勢へ変換する機能を提供する。
    /// </summary>
    public class WorldAlignedAnchor : MonoBehaviour
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
        /// 指定のワールド姿勢にアンカーを設置する。
        /// up は常にワールドの鉛直、向きは <paramref name="controllerWorldRotation"/> の水平成分。
        /// </summary>
        public void Place(Vector3 worldPosition, Quaternion controllerWorldRotation)
        {
            transform.SetPositionAndRotation(worldPosition, WorldAlignedYaw(controllerWorldRotation));
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

        /// <summary>回転の forward を水平面へ投影し、up をワールド鉛直に固定した回転を返す。</summary>
        static Quaternion WorldAlignedYaw(Quaternion rotation)
        {
            Vector3 forward = rotation * Vector3.forward;
            forward.y = 0f;
            if (forward.sqrMagnitude < 1e-6f)
            {
                // ほぼ真上／真下を向いている場合は up ベクトルを水平の手掛かりにする
                forward = rotation * Vector3.up;
                forward.y = 0f;
                if (forward.sqrMagnitude < 1e-6f)
                    forward = Vector3.forward;
            }

            return Quaternion.LookRotation(forward.normalized, Vector3.up);
        }

        void SetVisualVisible(bool visible)
        {
            if (_visual != null)
                _visual.SetActive(visible);
        }
    }
}
