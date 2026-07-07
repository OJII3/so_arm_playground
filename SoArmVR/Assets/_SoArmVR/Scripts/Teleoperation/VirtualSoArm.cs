using System.Collections.Generic;
using SoArmVR.Kinematics;
using UnityEngine;

namespace SoArmVR.Teleoperation
{
    public class VirtualSoArm : MonoBehaviour
    {
        [SerializeField] TextAsset _urdfAsset;
        [SerializeField] Material _defaultMaterial;
        [SerializeField] Material _ikSuccessMaterial;
        [SerializeField] Material _ikFailMaterial;
        [SerializeField] string _meshResourcesPath = "meshes/so101";

        public bool IsPlaced { get; private set; }
        public bool IsIkSolved { get; private set; }
        public double[] CurrentJoints { get; private set; } = new double[6];
        public UrdfModel Model { get; private set; }
        public double[] QSolution { get; set; } = new double[6];
        public Vector3 EeTargetLocalRos { get; set; }

        readonly Dictionary<string, Transform> _linkTransforms = new();
        readonly Dictionary<string, Mesh> _meshCache = new();
        Transform _rosToUnity;
        Material _fallbackMat;

        void Awake()
        {
            if (_urdfAsset != null)
                Model = UrdfModel.FromTextAsset(_urdfAsset);

            _fallbackMat = _defaultMaterial != null ? _defaultMaterial : new Material(Shader.Find("Universal Render Pipeline/Lit"));

            var rtgo = new GameObject("RosToUnity");
            _rosToUnity = rtgo.transform;
            _rosToUnity.SetParent(transform);

            if (Model != null)
                BuildFromUrdf();

            QSolution = new double[6];
            CurrentJoints = new double[6];
            SetGhostVisible(false);
        }

        void BuildFromUrdf()
        {
            for (int li = 0; li < Model.Links.Count; li++)
            {
                var link = Model.Links[li];
                var linkGo = new GameObject(link.name);
                var linkT = linkGo.transform;
                linkT.SetParent(_rosToUnity);
                _linkTransforms[link.name] = linkT;

                if (link.visuals.Count == 0)
                {
                    var marker = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    marker.transform.SetParent(linkT);
                    marker.transform.localScale = Vector3.one * 0.02f;
                    marker.name = "_fallback";
                    var mr = marker.GetComponent<MeshRenderer>();
                    if (mr != null) mr.material = _fallbackMat;
                    continue;
                }

                for (int vi = 0; vi < link.visuals.Count; vi++)
                {
                    var visual = link.visuals[vi];
                    GameObject visGo = null;

                    switch (visual.geometry.type)
                    {
                        case UrdfGeometryType.Mesh:
                            visGo = BuildMeshVisual(visual);
                            break;
                        case UrdfGeometryType.Box:
                            visGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
                            visGo.transform.localScale = visual.geometry.size;
                            break;
                        case UrdfGeometryType.Cylinder:
                            visGo = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                            visGo.transform.localScale = new Vector3(
                                visual.geometry.radius * 2f,
                                visual.geometry.length * 0.5f,
                                visual.geometry.radius * 2f);
                            break;
                        case UrdfGeometryType.Sphere:
                            visGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                            visGo.transform.localScale = Vector3.one * visual.geometry.radius * 2f;
                            break;
                    }

                    if (visGo == null)
                    {
                        visGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        visGo.transform.localScale = Vector3.one * 0.02f;
                    }

                    visGo.transform.SetParent(linkT);
                    visGo.name = $"visual_{vi}";
                    ApplyOrigin(visGo.transform, visual.origin);

                    var mr = visGo.GetComponent<MeshRenderer>();
                    if (mr != null) mr.material = _fallbackMat;
                }
            }
        }

        GameObject BuildMeshVisual(UrdfVisual visual)
        {
            var filename = System.IO.Path.GetFileName(visual.geometry.meshFilename);
            var mesh = LoadMesh(filename);
            if (mesh == null)
            {
                Debug.LogWarning($"Failed to load mesh '{filename}' for visual, using fallback");
                return null;
            }

            var go = new GameObject("mesh");
            var mf = go.AddComponent<MeshFilter>();
            mf.sharedMesh = mesh;
            go.AddComponent<MeshRenderer>();
            return go;
        }

        Mesh LoadMesh(string filename)
        {
            if (_meshCache.TryGetValue(filename, out var cached))
                return cached;

            var resourcePath = _meshResourcesPath + "/" + filename;
            var textAsset = Resources.Load<TextAsset>(resourcePath);
            if (textAsset == null)
            {
                Debug.LogWarning($"Mesh resource not found: {resourcePath}");
                _meshCache[filename] = null;
                return null;
            }

            var mesh = UrdfStlParser.ParseBinary(textAsset.bytes);
            if (mesh != null)
                mesh.name = filename;
            _meshCache[filename] = mesh;
            return mesh;
        }

        static void ApplyOrigin(Transform t, UrdfOrigin origin)
        {
            t.localPosition = origin.xyz;
            t.localRotation = Quaternion.Euler(
                Mathf.Rad2Deg * origin.rpy.x,
                Mathf.Rad2Deg * origin.rpy.y,
                Mathf.Rad2Deg * origin.rpy.z);
        }

        public void Place(Vector3 worldPosition, Quaternion worldRotation)
        {
            transform.SetPositionAndRotation(worldPosition, worldRotation);
            IsPlaced = true;
            SetGhostVisible(true);
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
            CurrentJoints[0] = j1; CurrentJoints[1] = j2; CurrentJoints[2] = j3;
            CurrentJoints[3] = j4; CurrentJoints[4] = j5; CurrentJoints[5] = j6;

            if (Model == null) return;
            UrdfKinematics.ComputeLinkTransforms(Model, CurrentJoints, out var linkTforms);

            for (int i = 0; i < Model.Links.Count && i < linkTforms.Length; i++)
            {
                var linkName = Model.Links[i].name;
                if (!_linkTransforms.TryGetValue(linkName, out var t)) continue;
                var m = linkTforms[i];
                t.localPosition = new Vector3(m.m03, m.m13, m.m23);
                t.localRotation = m.rotation;
            }
        }

        public void SetIkVisual(bool solved)
        {
            IsIkSolved = solved;
            var mat = solved ? _ikSuccessMaterial : _ikFailMaterial;
            if (mat == null) return;
            foreach (var kv in _linkTransforms)
            {
                foreach (var renderer in kv.Value.GetComponentsInChildren<Renderer>())
                    renderer.material = mat;
            }
        }

        void SetGhostVisible(bool visible)
        {
            if (_rosToUnity != null)
                _rosToUnity.gameObject.SetActive(visible);
        }

        public void SolveAndUpdate(double j4, double j5, double j6)
        {
            bool solved = UrdfIKSolver.Solve(Model, EeTargetLocalRos, QSolution, out var qSolved);
            if (solved)
            {
                QSolution = qSolved;
                SetIkVisual(true);
            }
            else
            {
                SetIkVisual(false);
            }
            QSolution[3] = j4; QSolution[4] = j5; QSolution[5] = j6;
            UpdateGhost(QSolution[0], QSolution[1], QSolution[2], QSolution[3], QSolution[4], QSolution[5]);
        }
    }
}
