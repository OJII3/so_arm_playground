using UnityEngine;
using ROSettaDDS.Dds;
using ROSettaDDS.Dds.QoS;
using ROSettaDDS.Msgs.Std;
using ROSettaDDS.Msgs.BuiltinInterfaces;
using ROSettaDDS.Msgs.Trajectory;
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
                return;
            }

            var now = System.DateTimeOffset.UtcNow;
            var stamp = new RosTime(
                (int)now.ToUnixTimeSeconds(),
                (uint)(now.Millisecond * 1_000_000));

            int sec = (int)_trajectoryDt;
            uint nsec = (uint)((_trajectoryDt - sec) * 1e9);
            var duration = new RosDuration(sec, nsec);

            var point = new JointTrajectoryPoint(
                new double[] {
                    _virtualSoArm.CurrentJoints[0],
                    _virtualSoArm.CurrentJoints[1],
                    _virtualSoArm.CurrentJoints[2],
                    _virtualSoArm.CurrentJoints[3],
                    _virtualSoArm.CurrentJoints[4],
                },
                new double[0],
                new double[0],
                new double[0],
                duration);

            var armTraj = new JointTrajectory(
                new Header(stamp, "follower"),
                new string[] { "1", "2", "3", "4", "5" },
                new JointTrajectoryPoint[] { point });

            _ = _armPub.PublishAsync(armTraj);

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
