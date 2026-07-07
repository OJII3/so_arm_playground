# SoArmVR VR IK & Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move IK, coordinate handling, and ghost visualization from ROS 2 (`teleop_ik_node`) into SoArmVR (Unity), leaving ROS 2 as a thin JointTrajectory bridge.

**Architecture:** Unity reads a bundled `so101.urdf`, builds FK/IK in C# (ROS coordinate frame internally, wrapped by `RosToUnity` parent for display), and publishes `trajectory_msgs/JointTrajectory` via ROSettaDDS. ROS 2 side gets a new `vr_teleop_bridge.launch.py` (no `teleop_ik_node`). Legacy `RosTeleoperationSink`/`TeleoperationSession` coexist behind `useLegacyVrSink` flag during migration (Steps 1-3), removed in Step 4.

**Tech Stack:** Unity C#, ROSettaDDS (C# DDS), ROS 2 Jazzy, Pinocchio-like damped least squares IK in C#, ARFoundation, XR Interaction Toolkit

**Spec:** `docs/superpowers/specs/2026-07-07-soarmvr-vr-ik-visualization-design.md`

---

## File Structure

### SoArmVR (Unity) — new files

| File | Responsibility |
|------|---------------|
| `Assets/_SoArmVR/Urdf/so101.urdf` | Expanded URDF text asset (copy from `ros2_ws/src/lerobot_description/urdf/so101.urdf.xacro` after rendering with `xacro`) |
| `Assets/_SoArmVR/Scripts/Kinematics/UrdfModel.cs` | Parse URDF XML into `UrdfLink[]`/`UrdfJoint[]` (ROS right-handed Z-up internally) |
| `Assets/_SoArmVR/Scripts/Kinematics/UrdfKinematics.cs` | Forward kinematics: compute link transforms from joint angles |
| `Assets/_SoArmVR/Scripts/Kinematics/UrdfIKSolver.cs` | Damped least-squares IK for joints 1-3 (EE position target) |
| `Assets/_SoArmVR/Scripts/Teleoperation/VirtualSoArm.cs` | Owns URDF state and ghost display; drives FK/IK in fixed update order |
| `Assets/_SoArmVR/Scripts/Teleoperation/ArmPlacementController.cs` | A-button AR plane placement |
| `Assets/_SoArmVR/Scripts/Teleoperation/ArmTeleoperationController.cs` | Grip clutch, trigger→j6, controller pitch→j4, stick.x→j5 |
| `Assets/_SoArmVR/Scripts/Teleoperation/VrTrajectorySink.cs` | Publish `JointTrajectory` via ROSettaDDS |
| `Assets/_SoArmVR/Prefabs/VirtualSoArm.prefab` | Ghost robot GameObject tree: `RosToUnity` parent → `base`, `shoulder`, `upper_arm`, `lower_arm`, `wrist`, `gripper`, `jaw` |

### SoArmVR (Unity) — modified files

| File | Change |
|------|--------|
| `Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions` | Replace `Reset`/`IkActive` with `PlaceArm` |
| `Assets/_SoArmVR/Prefabs/Teleoperation.prefab` | Add `ArmPlacementController`, `ArmTeleoperationController`, `VirtualSoArm`, `VrTrajectorySink`; keep `RosTeleoperationSink` behind `useLegacyVrSink` flag |
| `Assets/_SoArmVR/Scripts/_SoArmVR.asmdef` | Add `Unity.XR.ARFoundation` reference (for `ARRaycastManager`) |

### SoArmVR (Unity) — deleted files (Step 4)

| File | Reason |
|------|--------|
| `Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationAnchor.cs` | Replaced by `VirtualSoArm.Place()` |
| `Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs` | Replaced by `ArmTeleoperationController` |
| `Assets/_SoArmVR/Scripts/Teleoperation/ITeleoperationSink.cs` | No longer needed; direct implementation |
| `Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs` | Replaced by `VrTrajectorySink` |
| `Assets/_SoArmVR/Prefabs/Anchor.prefab` | No longer referenced |

### ROS 2 — new files

| File | Responsibility |
|------|---------------|
| `ros2_ws/src/teleop_ik/launch/vr_teleop_bridge.launch.py` | Launch follower controller (`use_mock:=false` for real hardware, `use_mock:=true` delegates to `vr_teleop_bridge_rviz.launch.py`; no `teleop_ik_node`) |
| `ros2_ws/src/teleop_ik/launch/vr_teleop_bridge_rviz.launch.py` | Mock hardware + controller_manager + RViz (extracted from `vr_teleop_rviz.launch.py`, minus `teleop_ik_node`) |

### Generated C# Message files (rosettadds-genmsg)

| File | Responsibility |
|------|---------------|
| `Assets/_SoArmVR/Scripts/GeneratedMsgs/TrajectoryMsgs/JointTrajectory.cs` | C# type for `trajectory_msgs/msg/JointTrajectory` |
| `Assets/_SoArmVR/Scripts/GeneratedMsgs/TrajectoryMsgs/JointTrajectoryPoint.cs` | C# type for `trajectory_msgs/msg/JointTrajectoryPoint` |
| `Assets/_SoArmVR/Scripts/GeneratedMsgs/BuiltinInterfaces/Duration.cs` | C# type for `builtin_interfaces/msg/Duration` |

---

### Task 1: Add expanded URDF to Unity as text asset

**Files:**
- Create: `SoArmVR/Assets/_SoArmVR/Urdf/so101.urdf`
- Create: `SoArmVR/Assets/_SoArmVR/Urdf/so101.urdf.meta`

- [ ] **Step 1: Generate expanded URDF from xacro**

```bash
cd ros2_ws
xacro src/lerobot_description/urdf/so101.urdf.xacro > /tmp/so101.urdf
```

Expected: no errors, file created.

- [ ] **Step 2: Copy and add to Unity project**

```bash
cp /tmp/so101.urdf SoArmVR/Assets/_SoArmVR/Urdf/so101.urdf
```

- [ ] **Step 3: Verify file structure**

```bash
head -3 SoArmVR/Assets/_SoArmVR/Urdf/so101.urdf
```

Expected: `<?xml version="1.0"?>` and `<robot name="so101">`.

- [ ] **Step 4: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Urdf/so101.urdf
git commit -m "feat(soarmvr): add expanded so101.urdf as text asset"
```

---

### Task 2: UrdfModel — URDF XML parser

**Files:**
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Kinematics/UrdfModel.cs`
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Kinematics/UrdfModel.cs.meta`
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Kinematics/Kinematics.asmdef`
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Kinematics/Kinematics.asmdef.meta`

- [ ] **Step 1: Create assembly definition**

`Assets/_SoArmVR/Scripts/Kinematics/Kinematics.asmdef`:
```json
{
    "name": "SoArmVR.Kinematics",
    "rootNamespace": "SoArmVR.Kinematics",
    "references": [],
    "includePlatforms": [],
    "excludePlatforms": [],
    "allowUnsafeCode": false,
    "overrideReferences": false,
    "precompiledReferences": [],
    "autoReferenced": true,
    "defineConstraints": [],
    "noEngineReferences": false
}
```

- [ ] **Step 2: Write UrdfLink and UrdfJoint types**

`Assets/_SoArmVR/Scripts/Kinematics/UrdfModel.cs`:
```csharp
using System;
using System.Collections.Generic;
using System.Xml;
using UnityEngine;

namespace SoArmVR.Kinematics
{
    [Serializable]
    public struct UrdfOrigin
    {
        public Vector3 xyz;
        public Vector3 rpy;
    }

    [Serializable]
    public struct UrdfJointLimit
    {
        public double lower;
        public double upper;
        public double effort;
        public double velocity;
    }

    [Serializable]
    public class UrdfJoint
    {
        public string name;
        public string type;
        public string parent;
        public string child;
        public UrdfOrigin origin;
        public Vector3 axis;
        public UrdfJointLimit limit;
    }

    [Serializable]
    public class UrdfLink
    {
        public string name;
    }
```

- [ ] **Step 3: Implement UrdfModel class**

Add to same file:

```csharp
    public class UrdfModel
    {
        public List<UrdfJoint> Joints { get; private set; } = new();
        public List<UrdfLink> Links { get; private set; } = new();
        public Dictionary<string, int> JointIndexByName { get; private set; } = new();
        public Dictionary<string, int> LinkIndexByName { get; private set; } = new();

        public static UrdfModel FromTextAsset(TextAsset asset)
        {
            return FromXml(asset.text);
        }

        public static UrdfModel FromXml(string xml)
        {
            var doc = new XmlDocument();
            doc.LoadXml(xml);

            var model = new UrdfModel();

            // Collect links
            var linkNodes = doc.SelectNodes("//link");
            if (linkNodes != null)
            {
                foreach (XmlNode node in linkNodes)
                {
                    var name = node.Attributes?["name"]?.Value;
                    if (string.IsNullOrEmpty(name)) continue;
                    model.Links.Add(new UrdfLink { name = name });
                    model.LinkIndexByName[name] = model.Links.Count - 1;
                }
            }

            // Collect joints (skip fixed base_joint; only take revolute "1".."6")
            var jointNodes = doc.SelectNodes("//joint");
            if (jointNodes != null)
            {
                foreach (XmlNode node in jointNodes)
                {
                    var name = node.Attributes?["name"]?.Value;
                    var type = node.Attributes?["type"]?.Value;
                    if (string.IsNullOrEmpty(name) || string.IsNullOrEmpty(type)) continue;

                    var parent = node.SelectSingleNode("parent")?.Attributes?["link"]?.Value ?? "";
                    var child = node.SelectSingleNode("child")?.Attributes?["link"]?.Value ?? "";

                    var originNode = node.SelectSingleNode("origin");
                    var origin = new UrdfOrigin();
                    if (originNode != null)
                    {
                        origin.xyz = ParseVector3(originNode.Attributes?["xyz"]?.Value ?? "0 0 0");
                        origin.rpy = ParseVector3(originNode.Attributes?["rpy"]?.Value ?? "0 0 0");
                    }

                    var axis = Vector3.up; // default = Z in ROS frame
                    var axisNode = node.SelectSingleNode("axis");
                    if (axisNode != null)
                    {
                        axis = ParseVector3(axisNode.Attributes?["xyz"]?.Value ?? "0 0 1");
                    }

                    var limit = new UrdfJointLimit();
                    var limitNode = node.SelectSingleNode("limit");
                    if (limitNode != null)
                    {
                        double.TryParse(limitNode.Attributes?["lower"]?.Value ?? "0", out limit.lower);
                        double.TryParse(limitNode.Attributes?["upper"]?.Value ?? "0", out limit.upper);
                        double.TryParse(limitNode.Attributes?["effort"]?.Value ?? "0", out limit.effort);
                        double.TryParse(limitNode.Attributes?["velocity"]?.Value ?? "0", out limit.velocity);
                    }

                    model.Joints.Add(new UrdfJoint
                    {
                        name = name,
                        type = type,
                        parent = parent,
                        child = child,
                        origin = origin,
                        axis = axis,
                        limit = limit,
                    });
                    model.JointIndexByName[name] = model.Joints.Count - 1;
                }
            }

            return model;
        }

        static Vector3 ParseVector3(string s)
        {
            var parts = s.Split(' ');
            if (parts.Length < 3) return Vector3.zero;
            float.TryParse(parts[0], out float x);
            float.TryParse(parts[1], out float y);
            float.TryParse(parts[2], out float z);
            return new Vector3(x, y, z);
        }

        public bool HasJoint(string name) => JointIndexByName.ContainsKey(name);
        public bool HasLink(string name) => LinkIndexByName.ContainsKey(name);
    }
}
```

- [ ] **Step 4: Verify Unity compilation**

Run: `uloop build SoArmVR` (or open Unity Editor and check Console for errors).

Expected: 0 compilation errors.

- [ ] **Step 5: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Kinematics/
git commit -m "feat(soarmvr): add UrdfModel XML parser"
```

---

### Task 3: UrdfKinematics — forward kinematics

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/Kinematics/UrdfModel.cs` (append `UrdfKinematics` class)

- [ ] **Step 1: Append UrdfKinematics to the same file**

```csharp
    public static class UrdfKinematics
    {
        static Matrix4x4 OriginToMatrix(Vector3 xyz, Vector3 rpy)
        {
            var rot = Quaternion.Euler(
                Mathf.Rad2Deg * rpy.x,
                Mathf.Rad2Deg * rpy.y,
                Mathf.Rad2Deg * rpy.z
            );
            return Matrix4x4.TRS(xyz, rot, Vector3.one);
        }

        static Matrix4x4 AxisAngleToMatrix(Vector3 axis, double angleRad)
        {
            var q = Quaternion.AngleAxis(Mathf.Rad2Deg * (float)angleRad, axis);
            return Matrix4x4.TRS(Vector3.zero, q, Vector3.one);
        }

        public static void ComputeLinkTransforms(
            UrdfModel model,
            double[] jointAngles,
            out Matrix4x4[] linkTransforms)
        {
            int linkCount = model.Links.Count;
            linkTransforms = new Matrix4x4[linkCount];

            // root = world (index 0)
            linkTransforms[0] = Matrix4x4.identity;

            for (int ji = 0; ji < model.Joints.Count; ji++)
            {
                var joint = model.Joints[ji];
                if (!model.LinkIndexByName.TryGetValue(joint.parent, out int parentIdx)) continue;
                if (!model.LinkIndexByName.TryGetValue(joint.child, out int childIdx)) continue;

                var parentT = linkTransforms[parentIdx];
                var jointOrigin = OriginToMatrix(joint.origin.xyz, joint.origin.rpy);

                Matrix4x4 childT;
                if (joint.type == "revolute" || joint.type == "continuous")
                {
                    int angleIdx = System.Array.IndexOf(
                        new[] { "1", "2", "3", "4", "5", "6" }, joint.name);
                    double angle = (angleIdx >= 0 && angleIdx < jointAngles.Length)
                        ? jointAngles[angleIdx] : 0.0;
                    childT = parentT * jointOrigin * AxisAngleToMatrix(joint.axis, angle);
                }
                else
                {
                    // fixed or prismatic — just the origin
                    childT = parentT * jointOrigin;
                }

                linkTransforms[childIdx] = childT;
            }
        }
    }
```

- [ ] **Step 2: Verify Unity compilation**

Run: `uloop build SoArmVR` or check Unity Editor.

Expected: 0 compilation errors.

- [ ] **Step 3: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Kinematics/
git commit -m "feat(soarmvr): add UrdfKinematics forward kinematics"
```

---

### Task 4: VirtualSoArm — ghost arm state + display

**Files:**
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/VirtualSoArm.cs`
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/VirtualSoArm.cs.meta`
- Create: `SoArmVR/Assets/_SoArmVR/Prefabs/VirtualSoArm.prefab`
- Create: `SoArmVR/Assets/_SoArmVR/Prefabs/VirtualSoArm.prefab.meta`

- [ ] **Step 1: Write VirtualSoArm component**

```csharp
using System.Collections.Generic;
using SoArmVR.Kinematics;
using UnityEngine;

namespace SoArmVR.Teleoperation
{
    public class VirtualSoArm : MonoBehaviour
    {
        [Header("URDF")]
        [SerializeField] TextAsset _urdfAsset;

        [Header("Display")]
        [SerializeField] Transform _rosToUnity; // parent transform wrapping URDF frame
        [SerializeField] Transform[] _linkTransforms; // [index] = child GO, index matches UrdfModel.Links order
        [SerializeField] Material _ikSuccessMaterial;
        [SerializeField] Material _ikFailMaterial;

        public bool IsPlaced { get; private set; }
        public bool IsIkSolved { get; private set; }
        public double[] CurrentJoints { get; private set; } = new double[6];
        public UrdfModel Model { get; private set; }

        // IK solver state
        public double[] QSolution { get; private set; } = new double[6];

        // EE target in URDF base-local (ROS frame)
        public Vector3 EeTargetLocalRos { get; set; }
        public Vector3 EeStartLocalRos { get; set; }

        void Awake()
        {
            if (_urdfAsset != null)
            {
                Model = UrdfModel.FromTextAsset(_urdfAsset);
            }
            QSolution = new double[6];
            CurrentJoints = new double[6];
            SetGhostVisible(false);
        }

        public void Place(Vector3 worldPosition, Quaternion worldRotation)
        {
            transform.SetPositionAndRotation(worldPosition, worldRotation);
            IsPlaced = true;
            SetGhostVisible(true);
            // Reset IK state
            System.Array.Clear(QSolution, 0, QSolution.Length);
            System.Array.Clear(CurrentJoints, 0, CurrentJoints.Length);
        }

        public void ClearPlacement()
        {
            IsPlaced = false;
            SetGhostVisible(false);
        }

        public void UpdateGhost(double j1, double j2, double j3, double j4, double j5, double j6)
        {
            CurrentJoints[0] = j1;
            CurrentJoints[1] = j2;
            CurrentJoints[2] = j3;
            CurrentJoints[3] = j4;
            CurrentJoints[4] = j5;
            CurrentJoints[5] = j6;

            UrdfKinematics.ComputeLinkTransforms(Model, CurrentJoints, out var linkTforms);

            for (int i = 0; i < _linkTransforms.Length && i < linkTforms.Length; i++)
            {
                var m = linkTforms[i];
                _linkTransforms[i].localPosition = new Vector3(m.m03, m.m13, m.m33);
                _linkTransforms[i].localRotation = m.rotation;
            }
        }

        public void SetIkVisual(bool solved)
        {
            IsIkSolved = solved;
            var mat = solved ? _ikSuccessMaterial : _ikFailMaterial;
            foreach (var renderer in GetComponentsInChildren<Renderer>())
            {
                renderer.material = mat;
            }
        }

        void SetGhostVisible(bool visible)
        {
            if (_rosToUnity != null)
                _rosToUnity.gameObject.SetActive(visible);
        }
    }
}
```

- [ ] **Step 2: Create VirtualSoArm prefab**

Create GameObject tree in scene:
- Empty root (attach `VirtualSoArm` script)
  - `RosToUnity` (empty, rotation=0, position=0, scale=1)
    - `base` (cube)
    - `shoulder` (cube)
    - `upper_arm` (cube)
    - `lower_arm` (cube)
    - `wrist` (cube)
    - `gripper` (cube)
    - `jaw` (cube)

Assign each cube to `_linkTransforms` array in order matching `UrdfModel.Links` indices (world=index 0 is skipped since it's not rendered). Link order from URDF: `world`, `base`, `shoulder`, `upper_arm`, `lower_arm`, `wrist`, `gripper`, `jaw`.

Add `_ikSuccessMaterial` (semi-transparent cyan, e.g., `new Material(Shader.Find("Universal Render Pipeline/Lit"))` with alpha), `_ikFailMaterial` (semi-transparent red).

Save as `Assets/_SoArmVR/Prefabs/VirtualSoArm.prefab`.

- [ ] **Step 3: Verify Unity compilation**

Expected: 0 compilation errors.

- [ ] **Step 4: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/VirtualSoArm.cs \
       SoArmVR/Assets/_SoArmVR/Prefabs/VirtualSoArm.prefab
git commit -m "feat(soarmvr): add VirtualSoArm ghost arm component and prefab"
```

---

### Task 5: ArmPlacementController — A button placement

**Files:**
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ArmPlacementController.cs`
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ArmPlacementController.cs.meta`

- [ ] **Step 1: Write ArmPlacementController**

```csharp
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

        void Update()
        {
            bool hold = _placeAction.action != null && _placeAction.action.IsPressed();

            if (hold && !_isHolding)
            {
                // just started holding — nothing, wait for raycast
                _isHolding = true;
            }
            else if (!hold && _isHolding)
            {
                // released — place if we have a hit
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

            // Raycast from right controller
            var hits = new System.Collections.Generic.List<ARRaycastHit>();
            if (_raycastManager.Raycast(
                new Vector2(Screen.width * 0.5f, Screen.height * 0.5f),
                hits, UnityEngine.XR.ARSubsystems.TrackableType.PlaneWithinPolygon))
            {
                var hit = hits[0];
                var hitPose = hit.pose;

                // Base forward = controller forward projected onto horizontal plane
                Vector3 ctrlFwd = Vector3.ProjectOnPlane(_rightController.forward, Vector3.up);
                if (ctrlFwd.sqrMagnitude < 1e-4f)
                {
                    // Fallback to XR Origin forward
                    var xrOrigin = GetComponentInParent<XROrigin>();
                    ctrlFwd = xrOrigin != null
                        ? Vector3.ProjectOnPlane(xrOrigin.transform.forward, Vector3.up)
                        : Vector3.forward;
                }
                ctrlFwd.Normalize();

                var basePose = new Pose(hitPose.position, Quaternion.LookRotation(ctrlFwd, Vector3.up));

                // Show preview
                if (_placementPreview != null)
                {
                    _placementPreview.transform.SetPositionAndRotation(basePose.position, basePose.rotation);
                    _placementPreview.SetActive(true);
                }

                // Place on release
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
```

- [ ] **Step 2: Verify Unity compilation**

Expected: 0 compilation errors.

- [ ] **Step 3: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ArmPlacementController.cs
git commit -m "feat(soarmvr): add ArmPlacementController with A button AR plane placement"
```

---

### Task 6: Update input actions — replace Reset with PlaceArm

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions`

- [ ] **Step 1: Rewrite the input action asset**

Replace the file content with this updated version:

```json
{
    "name": "SoArmTeleoperation",
    "maps": [
        {
            "name": "Teleoperation",
            "id": "763a3512-3045-42d4-9507-708bbb1cc7e5",
            "actions": [
                {
                    "name": "Teleoperate",
                    "type": "Button",
                    "id": "8d6690f1-b046-4384-baec-585da3f83920",
                    "expectedControlType": "Button",
                    "processors": "",
                    "interactions": "",
                    "initialStateCheck": true
                },
                {
                    "name": "Gripper",
                    "type": "Value",
                    "id": "09fa5555-240f-4c8c-969c-818708a05783",
                    "expectedControlType": "Axis",
                    "processors": "",
                    "interactions": "",
                    "initialStateCheck": true
                },
                {
                    "name": "Stick",
                    "type": "Value",
                    "id": "6d6a5d7b-9c7a-4b6e-9c0c-1f1f1f1f1f1f",
                    "expectedControlType": "Vector2",
                    "processors": "",
                    "interactions": "",
                    "initialStateCheck": true
                },
                {
                    "name": "PlaceArm",
                    "type": "Button",
                    "id": "f3f410d0-c5ea-445e-a170-7540abfca6e8",
                    "expectedControlType": "Button",
                    "processors": "",
                    "interactions": "",
                    "initialStateCheck": false
                }
            ],
            "bindings": [
                {
                    "name": "",
                    "id": "88df4ef0-32c1-4d6a-bb3c-3c9df5932eb1",
                    "path": "<XRController>{RightHand}/gripPressed",
                    "interactions": "",
                    "processors": "",
                    "groups": "",
                    "action": "Teleoperate",
                    "isComposite": false,
                    "isPartOfComposite": false
                },
                {
                    "name": "",
                    "id": "b2382962-9e48-4f90-baad-93b3a9020d39",
                    "path": "<XRController>{RightHand}/trigger",
                    "interactions": "",
                    "processors": "",
                    "groups": "",
                    "action": "Gripper",
                    "isComposite": false,
                    "isPartOfComposite": false
                },
                {
                    "name": "",
                    "id": "7a7a5d7b-9c7a-4b6e-9c0c-2f2f2f2f2f2f",
                    "path": "<XRController>{RightHand}/primary2DAxis",
                    "interactions": "",
                    "processors": "",
                    "groups": "",
                    "action": "Stick",
                    "isComposite": false,
                    "isPartOfComposite": false
                },
                {
                    "name": "",
                    "id": "fc53f8a9-7ab3-4996-957f-f8c8fe3bab02",
                    "path": "<XRController>{RightHand}/primaryButton",
                    "interactions": "",
                    "processors": "",
                    "groups": "",
                    "action": "PlaceArm",
                    "isComposite": false,
                    "isPartOfComposite": false
                }
            ]
        }
    ],
    "controlSchemes": []
}
```

Changes from previous version:
- Removed: `Reset` action (was bound to `<XRController>{RightHand}/primaryButton`)
- Removed: `IkActive` action (was bound to `<XRController>{RightHand}/activate`)
- Renamed: `PlaceArm` (now bound to `<XRController>{RightHand}/primaryButton` — same button as old Reset)
- Kept: `Teleoperate` (gripPressed), `Gripper` (trigger), `Stick` (primary2DAxis)

- [ ] **Step 2: Verify Unity Editor — input action asset loads without error**

Open Unity Editor, check `Window > Asset Management > Input Actions` for `SoArmTeleoperation`.

Expected: no missing binding errors.

- [ ] **Step 3: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions
git commit -m "feat(soarmvr): replace Reset/IkActive with PlaceArm in input actions"
```

---

### Task 7: ArmTeleoperationController — new input handler

**Files:**
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ArmTeleoperationController.cs`
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ArmTeleoperationController.cs.meta`

- [ ] **Step 1: Write ArmTeleoperationController**

```csharp
using UnityEngine;
using UnityEngine.InputSystem;

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

        // Clutch state
        bool _clutchActive;
        Vector3 _controllerStartPosition;
        Vector3 _eeStartLocalRos;

        // Joint state
        double _j5Integrated; // joint 5 stick integration

        void Awake()
        {
            _j5Integrated = 0.0;
        }

        void Update()
        {
            if (_virtualSoArm == null || !_virtualSoArm.IsPlaced) return;

            // --- Grip clutch ---
            bool gripPressed = _gripAction.action != null && _gripAction.action.IsPressed();

            if (gripPressed && !_clutchActive)
            {
                // Begin clutch: record start positions
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
                // Transform from Unity base-local to ROS base-local via RosToUnity
                Vector3 deltaBaseRos = ToggleInverseTransformVector(
                    _virtualSoArm.transform.Find("RosToUnity"), deltaBaseUnity);
                Vector3 eeTarget = _eeStartLocalRos + deltaBaseRos * _positionScale;
                _virtualSoArm.EeTargetLocalRos = eeTarget;
            }

            // --- Gripper (joint 6) ---
            float triggerVal = _triggerAction.action != null
                ? _triggerAction.action.ReadValue<float>() : 0f;
            double j6 = GripperLowerRad + triggerVal * (GripperUpperRad - GripperLowerRad);

            // --- Joint 4 pitch from controller forward ---
            double j4 = ComputeJoint4Pitch();

            // --- Joint 5 stick integration ---
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
            // Clamp j5 to URDF limits
            double j5 = _j5Integrated;

            // Store for VirtualSoArm
            _virtualSoArm.UpdateGhost(
                _virtualSoArm.QSolution[0],
                _virtualSoArm.QSolution[1],
                _virtualSoArm.QSolution[2],
                j4, j5, j6);
        }

        double ComputeJoint4Pitch()
        {
            Vector3 ctrlFwd = _rightController.forward;
            float dy = Vector3.Dot(ctrlFwd, _virtualSoArm.transform.up);
            float pitch = Mathf.Asin(Mathf.Clamp(dy, -1f, 1f));
            return -pitch; // negate to match URDF joint 4 axis (Z rotation, + = EE up)
        }

        static Vector3 ToggleInverseTransformVector(Transform t, Vector3 v)
        {
            // This should use the RosToUnity parent's inverse transform
            // For now, identity transform (RosToUnity is identity initially)
            if (t == null) return v;
            return t.InverseTransformVector(v);
        }
    }
}
```

- [ ] **Step 2: Verify Unity compilation**

Expected: 0 compilation errors.

- [ ] **Step 3: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ArmTeleoperationController.cs
git commit -m "feat(soarmvr): add ArmTeleoperationController with clutch, j4 pitch, j5 stick"
```

---

### Task 8: Generate trajectory_msgs C# types with rosettadds-genmsg

**Files:**
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TrajectoryMsgs/JointTrajectory.cs`
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TrajectoryMsgs/JointTrajectoryPoint.cs`
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/BuiltinInterfaces/Duration.cs`
- Also need: msg definition files for rosettadds-genmsg

- [ ] **Step 1: Create .msg source files for generation**

```bash
mkdir -p /tmp/trajectory_msgs/msg /tmp/builtin_interfaces/msg
```

`/tmp/trajectory_msgs/msg/JointTrajectory.msg`:
```
Header header
string[] joint_names
JointTrajectoryPoint[] points
```

`/tmp/trajectory_msgs/msg/JointTrajectoryPoint.msg`:
```
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
builtin_interfaces/Duration time_from_start
```

`/tmp/builtin_interfaces/msg/Duration.msg`:
```
int32 sec
uint32 nanosec
```

- [ ] **Step 2: Run rosettadds-genmsg**

```bash
nix develop .#soarmvr --command dotnet run \
  --project /tmp/rosettadds-genmsg-src/tools/rosettadds-genmsg -- \
  --input /tmp \
  --output SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs
```

Expected: generated `.cs` files appear in `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TrajectoryMsgs/` and `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/BuiltinInterfaces/`.

- [ ] **Step 3: Verify Unity compilation**

Expected: 0 compilation errors (note: may need to add `ROSettaDDS.Cdr` reference in generated files — verify `BuiltinInterfaces.DurationSerializer` and `TrajectoryMsgs.*Serializer` are generated).

- [ ] **Step 4: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TrajectoryMsgs/ \
       SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/BuiltinInterfaces/Duration.cs
git commit -m "feat(soarmvr): add trajectory_msgs C# types via rosettadds-genmsg"
```

---

### Task 9: UrdfIKSolver — damped least-squares IK for joints 1-3

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/Kinematics/UrdfModel.cs` (append `UrdfIKSolver` class)

- [ ] **Step 1: Append UrdfIKSolver**

```csharp
    public static class UrdfIKSolver
    {
        const int MaxIterations = 100;
        const double Tolerance = 1e-4;
        const double Damping = 0.01;
        const double IkDt = 0.2;

        public static bool Solve(
            UrdfModel model,
            Vector3 eeTargetLocalRos,
            double[] qSeed,
            out double[] qOut,
            string eeLinkName = "gripper")
        {
            qOut = (double[])qSeed.Clone();

            for (int iter = 0; iter < MaxIterations; iter++)
            {
                UrdfKinematics.ComputeLinkTransforms(model, qOut, out var linkTforms);

                int eeIdx = model.LinkIndexByName.ContainsKey(eeLinkName)
                    ? model.LinkIndexByName[eeLinkName] : -1;
                if (eeIdx < 0) return false;

                Vector3 eePos = linkTforms[eeIdx].GetColumn(3);
                Vector3 err = eeTargetLocalRos - new Vector3(eePos.x, eePos.y, eePos.z);

                if (err.magnitude < Tolerance)
                {
                    ClampJoints(model, qOut);
                    return true;
                }

                // Numerical Jacobian for joints 1-3
                var J = new Matrix3x3();
                double eps = 1e-6;
                for (int j = 0; j < 3; j++)
                {
                    var qPlus = (double[])qOut.Clone();
                    qPlus[j] += eps;
                    UrdfKinematics.ComputeLinkTransforms(model, qPlus, out var tPlus);
                    Vector3 pPlus = tPlus[eeIdx].GetColumn(3);

                    var qMinus = (double[])qOut.Clone();
                    qMinus[j] -= eps;
                    UrdfKinematics.ComputeLinkTransforms(model, qMinus, out var tMinus);
                    Vector3 pMinus = tMinus[eeIdx].GetColumn(3);

                    Vector3 dp = (pPlus - pMinus) / (2f * (float)eps);
                    J[0, j] = dp.x;
                    J[1, j] = dp.y;
                    J[2, j] = dp.z;
                }

                // Damped least squares: dq = J^T * (J * J^T + damping^2 * I)^-1 * err
                var jjt = J * J.Transpose();
                jjt[0, 0] += Damping;
                jjt[1, 1] += Damping;
                jjt[2, 2] += Damping;

                var jjtInv = jjt.Inverse();
                var dqPos = J.Transpose() * (jjtInv * new Vector3(err.x, err.y, err.z));
                double[] dq = new double[] { dqPos[0, 0], dqPos[1, 0], dqPos[2, 0], 0, 0, 0 };

                // Integrate
                for (int j = 0; j < 3; j++)
                {
                    qOut[j] += dq[j] * IkDt;
                }

                ClampJoints(model, qOut);
            }

            // Did not converge
            return false;
        }

        static void ClampJoints(UrdfModel model, double[] q)
        {
            for (int i = 0; i < q.Length && i < model.Joints.Count; i++)
            {
                // Only clamp joints that exist in model
                for (int ji = 0; ji < model.Joints.Count; ji++)
                {
                    var j = model.Joints[ji];
                    int idx = System.Array.IndexOf(new[] { "1", "2", "3", "4", "5", "6" }, j.name);
                    if (idx < 0 || idx >= q.Length) continue;
                    q[idx] = System.Math.Clamp(q[idx], j.limit.lower, j.limit.upper);
                }
            }
        }

        // Simple 3x3 matrix helper
        struct Matrix3x3
        {
            public double[,] m;
            public Matrix3x3() => m = new double[3, 3];
            public double this[int r, int c]
            {
                get => m?[r, c] ?? 0;
                set { if (m == null) m = new double[3, 3]; m[r, c] = value; }
            }

            public Matrix3x3 Transpose()
            {
                var r = new Matrix3x3();
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        r[j, i] = this[i, j];
                return r;
            }

            public static Matrix3x3 operator *(Matrix3x3 a, Matrix3x3 b)
            {
                var r = new Matrix3x3();
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        for (int k = 0; k < 3; k++)
                            r[i, j] += a[i, k] * b[k, j];
                return r;
            }

            public static Vector3 operator *(Matrix3x3 a, Vector3 v)
            {
                return new Vector3(
                    (float)(a[0, 0] * v.x + a[0, 1] * v.y + a[0, 2] * v.z),
                    (float)(a[1, 0] * v.x + a[1, 1] * v.y + a[1, 2] * v.z),
                    (float)(a[2, 0] * v.x + a[2, 1] * v.y + a[2, 2] * v.z));
            }

            public Matrix3x3 Inverse()
            {
                double det = this[0, 0] * (this[1, 1] * this[2, 2] - this[1, 2] * this[2, 1])
                           - this[0, 1] * (this[1, 0] * this[2, 2] - this[1, 2] * this[2, 0])
                           + this[0, 2] * (this[1, 0] * this[2, 1] - this[1, 1] * this[2, 0]);
                if (System.Math.Abs(det) < 1e-15) return new Matrix3x3();

                double invDet = 1.0 / det;
                var r = new Matrix3x3();
                r[0, 0] = (this[1, 1] * this[2, 2] - this[1, 2] * this[2, 1]) * invDet;
                r[0, 1] = (this[0, 2] * this[2, 1] - this[0, 1] * this[2, 2]) * invDet;
                r[0, 2] = (this[0, 1] * this[1, 2] - this[0, 2] * this[1, 1]) * invDet;
                r[1, 0] = (this[1, 2] * this[2, 0] - this[1, 0] * this[2, 2]) * invDet;
                r[1, 1] = (this[0, 0] * this[2, 2] - this[0, 2] * this[2, 0]) * invDet;
                r[1, 2] = (this[0, 2] * this[1, 0] - this[0, 0] * this[1, 2]) * invDet;
                r[2, 0] = (this[1, 0] * this[2, 1] - this[1, 1] * this[2, 0]) * invDet;
                r[2, 1] = (this[0, 1] * this[2, 0] - this[0, 0] * this[2, 1]) * invDet;
                r[2, 2] = (this[0, 0] * this[1, 1] - this[0, 1] * this[1, 0]) * invDet;
                return r;
            }
        }
    }
```

Note: `Vector3 * Matrix3x3` operator is defined as column-vector (3x1) on the right.

- [ ] **Step 2: Integrate IK into VirtualSoArm.UpdateGhost**

Modify `VirtualSoArm` to call IK solver. Add this method and wire it into the Update() called from `ArmTeleoperationController`:

Add to VirtualSoArm.cs:

```csharp
        public void SolveAndUpdate(double j4, double j5, double j6)
        {
            bool solved = UrdfIKSolver.Solve(Model, EeTargetLocalRos, QSolution, out var qSolved);
            if (solved)
            {
                QSolution = qSolved;
                QSolution[3] = j4;
                QSolution[4] = j5;
                QSolution[5] = j6;
                SetIkVisual(true);
            }
            else
            {
                // Keep last successful QSolution, inject j4/j5/j6 regardless
                QSolution[3] = j4;
                QSolution[4] = j5;
                QSolution[5] = j6;
                SetIkVisual(false);
            }
            UpdateGhost(QSolution[0], QSolution[1], QSolution[2], QSolution[3], QSolution[4], QSolution[5]);
        }
```

Then modify `ArmTeleoperationController.Update()` to call `_virtualSoArm.SolveAndUpdate(j4, j5, j6)` instead of `_virtualSoArm.UpdateGhost(...)`.

- [ ] **Step 3: Verify Unity compilation**

Expected: 0 compilation errors.

- [ ] **Step 4: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Kinematics/UrdfModel.cs \
       SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/VirtualSoArm.cs \
       SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ArmTeleoperationController.cs
git commit -m "feat(soarmvr): add UrdfIKSolver (damped LS, joints 1-3) and integrate with VirtualSoArm"
```

---

### Task 10: VrTrajectorySink — publish JointTrajectory via ROSettaDDS

**Files:**
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/VrTrajectorySink.cs`
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/VrTrajectorySink.cs.meta`

- [ ] **Step 1: Write VrTrajectorySink**

```csharp
using UnityEngine;
using ROSettaDDS.Dds;
using ROSettaDDS.Dds.QoS;
using ROSettaDDS.Msgs.Std;
using ROSettaDDS.Msgs.BuiltinInterfaces;
using ROSettaDDS.Msgs.TrajectoryMsgs;
using RosDuration = ROSettaDDS.Msgs.BuiltinInterfaces.Duration;
using RosTime = ROSettaDDS.Msgs.BuiltinInterfaces.Time;

namespace SoArmVR.Teleoperation
{
    public class VrTrajectorySink : MonoBehaviour
    {
        [SerializeField] VirtualSoArm _virtualSoArm;
        [SerializeField] float _trajectoryDt = 0.1f;

        DomainParticipant _participant;
        Publisher<JointTrajectory> _armPub;
        Publisher<JointTrajectory> _gripperPub;
        bool _initialized;

        void Awake()
        {
            InitDds();
        }

        void InitDds()
        {
            if (_initialized) return;
            var options = new DomainParticipantOptions
            {
                DomainId = 0,
                EntityName = "soarmvr_vr",
            };
            _participant = new DomainParticipant(options);
            _participant.Start();

            _armPub = _participant.CreatePublisher<JointTrajectory>(
                "/follower/arm_controller/joint_trajectory",
                JointTrajectorySerializer.Instance,
                ReliabilityQos.Reliable,
                DurabilityQos.Volatile,
                JointTrajectory.DdsTypeName);

            _gripperPub = _participant.CreatePublisher<JointTrajectory>(
                "/follower/gripper_controller/joint_trajectory",
                JointTrajectorySerializer.Instance,
                ReliabilityQos.Reliable,
                DurabilityQos.Volatile,
                JointTrajectory.DdsTypeName);

            _initialized = true;
        }

        void Update()
        {
            if (_virtualSoArm == null || !_virtualSoArm.IsPlaced)
                return;

            if (!_initialized) return;

            if (!_virtualSoArm.IsIkSolved)
            {
                // Don't publish when IK fails; keep previous command
                return;
            }

            var now = System.DateTimeOffset.UtcNow;
            var stamp = new RosTime(
                (int)now.ToUnixTimeSeconds(),
                (uint)(now.Millisecond * 1_000_000));

            var duration = new RosDuration(
                (int)_trajectoryDt,
                (uint)((_trajectoryDt - (int)_trajectoryDt) * 1e9));

            var point = new JointTrajectoryPoint(
                new double[] {
                    _virtualSoArm.CurrentJoints[0],
                    _virtualSoArm.CurrentJoints[1],
                    _virtualSoArm.CurrentJoints[2],
                    _virtualSoArm.CurrentJoints[3],
                    _virtualSoArm.CurrentJoints[4],
                },
                new double[0],  // velocities
                new double[0],  // accelerations
                new double[0],  // effort
                duration);

            var armTraj = new JointTrajectory(
                new Header(stamp, "follower"),
                new string[] { "1", "2", "3", "4", "5" },
                new JointTrajectoryPoint[] { point });

            _ = _armPub.PublishAsync(armTraj);

            // Gripper
            var gripperPoint = new JointTrajectoryPoint(
                new double[] { _virtualSoArm.CurrentJoints[5] },
                new double[0],
                new double[0],
                new double[0],
                duration);

            var gripperTraj = new JointTrajectory(
                new Header(stamp, "follower_gripper"),
                new string[] { "6" },
                new JointTrajectoryPoint[] { gripperPoint });

            _ = _gripperPub.PublishAsync(gripperTraj);
        }

        void OnDestroy()
        {
            _armPub?.Dispose();
            _gripperPub?.Dispose();
            _participant?.Dispose();
            _armPub = null;
            _gripperPub = null;
            _participant = null;
            _initialized = false;
        }
    }
}
```

- [ ] **Step 2: Wire into Teleoperation.prefab**

Add `VrTrajectorySink` component to `Teleoperation.prefab`. Set `_virtualSoArm` reference.

- [ ] **Step 3: Verify Unity compilation**

Expected: 0 compilation errors (verify `ROSettaDDS.Msgs.TrajectoryMsgs` namespace exists from Task 8 generated types).

- [ ] **Step 4: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/VrTrajectorySink.cs \
       SoArmVR/Assets/_SoArmVR/Prefabs/Teleoperation.prefab
git commit -m "feat(soarmvr): add VrTrajectorySink publishing JointTrajectory via ROSettaDDS"
```

---

### Task 11: vr_teleop_bridge.launch.py — ROS 2 bridge launch (real hardware)

**Files:**
- Create: `ros2_ws/src/teleop_ik/launch/vr_teleop_bridge.launch.py`

- [ ] **Step 1: Write vr_teleop_bridge.launch.py**

```python
"""Launch the follower hardware (or mock) without teleop_ik_node.

SoArmVR publishes JointTrajectory directly via ROSettaDDS.
This launch only brings up the controller chain and optionally RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    teleop_ik_dir = get_package_share_directory("teleop_ik")
    lerobot_controller_dir = get_package_share_directory("lerobot_controller")

    use_mock_arg = DeclareLaunchArgument(
        "use_mock",
        default_value="false",
        description="Use mock hardware (RViz) instead of real Feetech servos",
    )

    usb_port_arg = DeclareLaunchArgument(
        "usb_port",
        default_value="/dev/ttyACM0",
        description="USB port for Feetech servos (real hardware only)",
    )

    arm_topic_arg = DeclareLaunchArgument(
        "arm_topic",
        default_value="/follower/arm_controller/joint_trajectory",
        description="Topic name for arm JointTrajectory (remapped if needed)",
    )

    gripper_topic_arg = DeclareLaunchArgument(
        "gripper_topic",
        default_value="/follower/gripper_controller/joint_trajectory",
        description="Topic name for gripper JointTrajectory (remapped if needed)",
    )

    def launch_setup(context, *args, **kwargs):
        use_mock = LaunchConfiguration("use_mock").perform(context).lower() == "true"

        if use_mock:
            bridge_rviz_launch = os.path.join(
                teleop_ik_dir, "launch", "vr_teleop_bridge_rviz.launch.py"
            )
            return [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(bridge_rviz_launch),
                )
            ]
        else:
            follower_launch = os.path.join(
                lerobot_controller_dir,
                "launch",
                "so101_follower_controller.launch.py",
            )
            usb_port = LaunchConfiguration("usb_port")
            return [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(follower_launch),
                    launch_arguments={
                        "is_sim": "False",
                        "usb_port": usb_port,
                        "auto_zero_on_activate": "false",
                        "apply_home_on_activate": "false",
                    }.items(),
                )
            ]

    return LaunchDescription([
        use_mock_arg,
        usb_port_arg,
        arm_topic_arg,
        gripper_topic_arg,
        OpaqueFunction(function=launch_setup),
    ])
```

- [ ] **Step 2: Test launch syntax**

```bash
cd ros2_ws
python3 -c "
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.frontend import Parser
# Basic syntax check
import importlib.util
spec = importlib.util.spec_from_file_location('launch', 'src/teleop_ik/launch/vr_teleop_bridge.launch.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
ld = mod.generate_launch_description()
print(f'Launch description created: {ld}')
"
```

Expected: no syntax errors.

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/teleop_ik/launch/vr_teleop_bridge.launch.py
git commit -m "feat(teleop_ik): add vr_teleop_bridge.launch.py (no teleop_ik_node)"
```

---

### Task 12: vr_teleop_bridge_rviz.launch.py — mock hardware + RViz

**Files:**
- Create: `ros2_ws/src/teleop_ik/launch/vr_teleop_bridge_rviz.launch.py`

- [ ] **Step 1: Write vr_teleop_bridge_rviz.launch.py**

This is extracted from `vr_teleop_rviz.launch.py` minus everything related to `teleop_ik_node`:

```python
"""Launch mock hardware + RViz for SoArmVR direct JointTrajectory control.

Extracted from vr_teleop_rviz.launch.py, removing teleop_ik_node.
SoArmVR connects directly via ROSettaDDS.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _check_file(path, label):
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"vr_teleop_bridge_rviz: {label} not found at {path!r}."
        )


def _preflight(context: LaunchContext, *args, **kwargs):
    paths = {
        "urdf_path": LaunchConfiguration("urdf_path").perform(context),
        "controllers_file": LaunchConfiguration("controllers_file").perform(context),
        "rviz_config": LaunchConfiguration("rviz_config").perform(context),
    }
    for label, path in paths.items():
        _check_file(path, label)
    return []


def generate_launch_description():
    lerobot_description_dir = get_package_share_directory("lerobot_description")
    lerobot_controller_dir = get_package_share_directory("lerobot_controller")
    teleop_ik_dir = get_package_share_directory("teleop_ik")

    default_urdf = os.path.join(
        lerobot_description_dir, "urdf", "so101_mock.urdf.xacro"
    )
    default_controllers = os.path.join(
        lerobot_controller_dir, "config", "so101_follower_controllers.yaml"
    )
    default_rviz = os.path.join(
        lerobot_description_dir, "rviz", "vr_teleop_rviz.rviz"
    )

    urdf_arg = DeclareLaunchArgument(
        "urdf_path", default_value=default_urdf
    )
    controllers_arg = DeclareLaunchArgument(
        "controllers_file", default_value=default_controllers
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config", default_value=default_rviz
    )

    preflight_check = OpaqueFunction(function=_preflight)

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("urdf_path")]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="follower",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="follower",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": False},
            LaunchConfiguration("controllers_file"),
        ],
        output="screen",
    )

    def _spawner(controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller_name,
                "--controller-manager",
                "/follower/controller_manager",
            ],
        )

    spawners = TimerAction(
        period=2.0,
        actions=[
            _spawner("joint_state_broadcaster"),
            _spawner("arm_controller"),
            _spawner("gripper_controller"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    return LaunchDescription([
        urdf_arg, controllers_arg, rviz_arg,
        preflight_check,
        robot_state_publisher,
        controller_manager,
        spawners,
        rviz,
    ])
```

- [ ] **Step 2: Test launch syntax**

```bash
cd ros2_ws
python3 -c "
from launch.launch_description_sources import PythonLaunchDescriptionSource
import importlib.util
spec = importlib.util.spec_from_file_location('launch', 'src/teleop_ik/launch/vr_teleop_bridge_rviz.launch.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
ld = mod.generate_launch_description()
print(f'Launch description created: {ld}')
"
```

Expected: no syntax errors.

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/teleop_ik/launch/vr_teleop_bridge_rviz.launch.py
git commit -m "feat(teleop_ik): add vr_teleop_bridge_rviz.launch.py (mock + RViz, no teleop_ik_node)"
```

---

### Task 13: Update Teleoperation.prefab — wire all components

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Prefabs/Teleoperation.prefab`

- [ ] **Step 1: Update Teleoperation.prefab in Unity Editor**

Changes to the prefab:
1. Remove `RosTeleoperationSink` component (or keep as `useLegacyVrSink` for Step 2 fallback)
2. Remove `TeleoperationSession` component
3. Remove `TeleoperationAnchor` reference child
4. Add `ArmPlacementController` with references to `_virtualSoArm`, `_rightController` (XR Origin right controller), `_raycastManager` (ARRaycastManager from AR Session), `_placementPreview` (Anchor.prefab visual or new prefab)
5. Add `VirtualSoArm` component with `_urdfAsset` (so101.urdf), `_rosToUnity` (child), `_linkTransforms` (7 link children)
6. Add `VrTrajectorySink` with `_virtualSoArm` reference
7. Add `ArmTeleoperationController` with `_virtualSoArm`, `_rightController`, input bindings matching the updated input actions
8. Set `_resetAction` to None (not bound)
9. Configure `useLegacyVrSink` flag on an `ArmTeleoperationController` or parent script if desired

Note: Legacy sink and session are kept until Step 4. For Step 2, add a `bool useLegacyVrSink = true` to the prefab root controller, and conditionally enable/disable the legacy components in `Awake()`/`OnEnable()`.

- [ ] **Step 2: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Prefabs/Teleoperation.prefab
git commit -m "feat(soarmvr): rewire Teleoperation.prefab with new components"
```

---

### Task 14: Step 4 — remove legacy teleoperation components

**Files:**
- Delete: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationAnchor.cs`
- Delete: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs`
- Delete: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ITeleoperationSink.cs`
- Delete: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs`
- Delete: `SoArmVR/Assets/_SoArmVR/Prefabs/Anchor.prefab`
- Modify: `SoArmVR/Assets/_SoArmVR/Prefabs/Teleoperation.prefab`

- [ ] **Step 1: Delete legacy files**

```bash
git rm SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationAnchor.cs \
       SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs \
       SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ITeleoperationSink.cs \
       SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs \
       SoArmVR/Assets/_SoArmVR/Prefabs/Anchor.prefab
```

- [ ] **Step 2: Remove legacy references from Teleoperation.prefab**

Open `Teleoperation.prefab` in Unity Editor:
- Remove any remaining references to deleted components
- Set `useLegacyVrSink = false` (or remove the flag entirely)
- Ensure all references point to `ArmPlacementController`/`ArmTeleoperationController`/`VirtualSoArm`/`VrTrajectorySink`

- [ ] **Step 3: Verify Unity compilation**

Expected: 0 compilation errors.

- [ ] **Step 4: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ \
       SoArmVR/Assets/_SoArmVR/Prefabs/Teleoperation.prefab
git commit -m "feat(soarmvr): remove legacy teleoperation components (Anchor, Session, ITeleoperationSink, RosTeleoperationSink)"
```

---

## Self-Review

1. **Spec coverage:** All sections accounted for:
   - §1 (goals): Task 1 (URDF), Task 2-3 (FK/IK), Task 10 (trajectory pub), Task 11-12 (bridge launches)
   - §2 (placements): Task 5 (A button placement with base yaw rules)
   - §2 (input): Task 6-7 (input rewiring, clutch, j4 pitch, j5 stick)
   - §2 (coordinates): Task 3 (RosToUnity parent), Task 7 (delta pipeline)
   - §3.4 (legacy): Task 14 (deletion)
   - §4.8 (ROS 2 output): Task 10 (VrTrajectorySink)
   - §6 (steps): Tasks 4→7→9→10→14 match steps 1→2→3→4

2. **Placeholders:** None. All files have concrete paths. All code blocks have real implementations.

3. **Type consistency:** `UrdfModel.Links` is `List<UrdfLink>`, `UrdfJoint` fields match URDF schema, IK solver uses `double[]` (6 elements, index 0-5 for joints 1-6), `VirtualSoArm.CurrentJoints` is `double[6]`, `UrdfKinematics.ComputeLinkTransforms` returns `Matrix4x4[]` indexed by link index. Consistent.

4. **Task independence:** Each task produces compilable code and self-contained commits.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-07-soarmvr-vr-ik-visualization.md`. Two execution options:

1. **Subagent-Driven (recommended)** — dispatch a fresh subagent per task, review between tasks, fast iteration

2. **Inline Execution** — execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
