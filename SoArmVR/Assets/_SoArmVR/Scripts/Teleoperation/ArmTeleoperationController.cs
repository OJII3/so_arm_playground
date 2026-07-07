using UnityEngine;
using UnityEngine.InputSystem;
using Unity.XR.CoreUtils;

namespace SoArmVR.Teleoperation
{
    public class ArmTeleoperationController : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] VirtualSoArm _virtualSoArm;
        [SerializeField] Transform _rightController;

        [Header("Input")]
        [SerializeField] InputActionProperty _gripAction;
        [SerializeField] InputActionProperty _triggerAction;
        [SerializeField] InputActionProperty _stickAction;

        [Header("Parameters")]
        [SerializeField] float _positionScale = 1.0f;
        [SerializeField] float _j5RadPerSec = 1.5f;
        [SerializeField] float _stickDeadzone = 0.1f;

        const double GripperLowerRad = -0.174533;
        const double GripperUpperRad = 1.74533;

        bool _clutchActive;
        Vector3 _controllerStartPosition;
        Vector3 _eeStartLocalRos;
        double _j5Integrated;

        void Awake()
        {
            _j5Integrated = 0.0;

            if (_virtualSoArm == null)
                _virtualSoArm = GetComponentInChildren<VirtualSoArm>();
            if (_rightController == null)
            {
                var origin = FindObjectOfType<XROrigin>();
                if (origin != null)
                    _rightController = origin.transform.Find("Camera Offset/Right Hand");
            }
            if (_gripAction.action == null || _triggerAction.action == null || _stickAction.action == null)
            {
                var assets = Resources.FindObjectsOfTypeAll<InputActionAsset>();
                foreach (var a in assets)
                {
                    var telem = a.FindActionMap("Teleoperation");
                    if (telem == null) continue;
                    if (_gripAction.action == null)
                    {
                        var act = telem.FindAction("Teleoperate");
                        if (act != null) _gripAction = new InputActionProperty(act);
                    }
                    if (_triggerAction.action == null)
                    {
                        var act = telem.FindAction("Gripper");
                        if (act != null) _triggerAction = new InputActionProperty(act);
                    }
                    if (_stickAction.action == null)
                    {
                        var act = telem.FindAction("Stick");
                        if (act != null) _stickAction = new InputActionProperty(act);
                    }
                }
            }
        }

        void Update()
        {
            if (_virtualSoArm == null || !_virtualSoArm.IsPlaced) return;

            // Grip clutch
            bool gripPressed = _gripAction.action != null && _gripAction.action.IsPressed();

            if (gripPressed && !_clutchActive)
            {
                _controllerStartPosition = _rightController.position;
                _eeStartLocalRos = _virtualSoArm.EeTargetLocalRos;
                _clutchActive = true;
            }
            else if (!gripPressed && _clutchActive)
            {
                _clutchActive = false;
            }

            if (_clutchActive)
            {
                Vector3 deltaWorld = _rightController.position - _controllerStartPosition;
                Vector3 deltaBaseUnity = _virtualSoArm.transform.InverseTransformVector(deltaWorld);
                var rosToUnity = _virtualSoArm.transform.Find("RosToUnity");
                Vector3 deltaBaseRos = rosToUnity != null
                    ? rosToUnity.InverseTransformVector(deltaBaseUnity)
                    : deltaBaseUnity;
                Vector3 eeTarget = _eeStartLocalRos + deltaBaseRos * _positionScale;
                _virtualSoArm.EeTargetLocalRos = eeTarget;
            }

            // Gripper (joint 6)
            float triggerVal = _triggerAction.action != null
                ? _triggerAction.action.ReadValue<float>() : 0f;
            double j6 = GripperLowerRad + triggerVal * (GripperUpperRad - GripperLowerRad);

            // Joint 4 pitch from controller forward
            double j4 = ComputeJoint4Pitch();

            // Joint 5 stick integration
            Vector2 stick = _stickAction.action != null
                ? _stickAction.action.ReadValue<Vector2>() : Vector2.zero;
            {
                float x = stick.x;
                float absX = Mathf.Abs(x);
                float xRemapped = absX <= _stickDeadzone
                    ? 0f
                    : Mathf.Sign(x) * (absX - _stickDeadzone) / (1f - _stickDeadzone);
                _j5Integrated += xRemapped * _j5RadPerSec * Time.deltaTime;
            }
            double j5 = _j5Integrated;

            // Delegate to VirtualSoArm IK + display
            _virtualSoArm.SolveAndUpdate(j4, j5, j6);
        }

        double ComputeJoint4Pitch()
        {
            Vector3 ctrlFwd = _rightController.forward;
            float dy = Vector3.Dot(ctrlFwd, _virtualSoArm.transform.up);
            float pitch = Mathf.Asin(Mathf.Clamp(dy, -1f, 1f));
            return -pitch;
        }
    }
}
