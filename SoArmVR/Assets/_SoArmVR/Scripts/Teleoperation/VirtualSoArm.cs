using SoArmVR.Kinematics;
using UnityEngine;

namespace SoArmVR.Teleoperation
{
    public class VirtualSoArm : MonoBehaviour
    {
        [Header("URDF")]
        [SerializeField] TextAsset _urdfAsset;

        [Header("Display")]
        [SerializeField] Transform _rosToUnity;
        [SerializeField] Transform[] _linkTransforms;
        [SerializeField] Material _ikSuccessMaterial;
        [SerializeField] Material _ikFailMaterial;

        public bool IsPlaced { get; private set; }
        public bool IsIkSolved { get; private set; }
        public double[] CurrentJoints { get; private set; } = new double[6];
        public UrdfModel Model { get; private set; }
        public double[] QSolution { get; set; } = new double[6];
        public Vector3 EeTargetLocalRos { get; set; }

        void Awake()
        {
            if (_urdfAsset != null)
                Model = UrdfModel.FromTextAsset(_urdfAsset);
            QSolution = new double[6];
            CurrentJoints = new double[6];
            SetGhostVisible(false);
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

            UrdfKinematics.ComputeLinkTransforms(Model, CurrentJoints, out var linkTforms);

            for (int i = 0; i < _linkTransforms.Length && i < linkTforms.Length; i++)
            {
                var m = linkTforms[i];
                _linkTransforms[i].localPosition = new Vector3(m.m03, m.m13, m.m23);
                _linkTransforms[i].localRotation = m.rotation;
            }
        }

        public void SetIkVisual(bool solved)
        {
            IsIkSolved = solved;
            var mat = solved ? _ikSuccessMaterial : _ikFailMaterial;
            if (mat == null) return;
            foreach (var renderer in GetComponentsInChildren<Renderer>())
                renderer.material = mat;
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
