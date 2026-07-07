using UnityEngine;
using UnityEngine.InputSystem;
using Unity.XR.CoreUtils;
using UnityEngine.XR.ARFoundation;

namespace SoArmVR.Teleoperation
{
    public class ArmPlacementController : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] VirtualSoArm _virtualSoArm;
        [SerializeField] Transform _rightController;
        [SerializeField] ARRaycastManager _raycastManager;
        [SerializeField] GameObject _placementPreview;

        [Header("Input")]
        [SerializeField] InputActionProperty _placeAction;

        bool _isHolding;
        bool _isPlaced;

        void Awake()
        {
            if (_raycastManager == null)
                _raycastManager = FindObjectOfType<ARRaycastManager>();
            if (_virtualSoArm == null)
                _virtualSoArm = GetComponentInChildren<VirtualSoArm>();
            if (_rightController == null)
            {
                var origin = FindObjectOfType<XROrigin>();
                if (origin != null)
                    _rightController = origin.transform.Find("Camera Offset/Right Hand");
            }
            if (_placeAction.action == null)
            {
                var assets = Resources.FindObjectsOfTypeAll<InputActionAsset>();
                foreach (var a in assets)
                {
                    var act = a.FindAction("PlaceArm");
                    if (act != null) { _placeAction = new InputActionProperty(act); break; }
                }
            }
        }

        void Update()
        {
            bool hold = _placeAction.action != null && _placeAction.action.IsPressed();

            if (hold && !_isHolding)
            {
                _isHolding = true;
            }
            else if (!hold && _isHolding)
            {
                if (_isPlaced)
                {
                    _isHolding = false;
                    _isPlaced = false;
                    return;
                }
            }
            _isHolding = hold;

            if (!hold)
            {
                if (_placementPreview != null)
                    _placementPreview.SetActive(false);
                return;
            }

            var hits = new System.Collections.Generic.List<ARRaycastHit>();
            if (_raycastManager.Raycast(
                new Vector2(Screen.width * 0.5f, Screen.height * 0.5f),
                hits, UnityEngine.XR.ARSubsystems.TrackableType.PlaneWithinPolygon))
            {
                var hit = hits[0];
                var hitPose = hit.pose;

                Vector3 ctrlFwd = Vector3.ProjectOnPlane(_rightController.forward, Vector3.up);
                if (ctrlFwd.sqrMagnitude < 1e-4f)
                {
                    var xrOrigin = GetComponentInParent<XROrigin>();
                    ctrlFwd = xrOrigin != null
                        ? Vector3.ProjectOnPlane(xrOrigin.transform.forward, Vector3.up)
                        : Vector3.forward;
                }
                ctrlFwd.Normalize();

                var basePose = new Pose(hitPose.position, Quaternion.LookRotation(ctrlFwd, Vector3.up));

                if (_placementPreview != null)
                {
                    _placementPreview.transform.SetPositionAndRotation(basePose.position, basePose.rotation);
                    _placementPreview.SetActive(true);
                }

                if (!hold && _isHolding)
                {
                    _virtualSoArm.Place(basePose.position, basePose.rotation);
                    _isPlaced = true;
                }
            }
            else
            {
                if (_placementPreview != null)
                    _placementPreview.SetActive(false);
            }
        }
    }
}
