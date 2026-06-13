using System.Net;
using UnityEngine;
using ROSettaDDS.Dds;
using ROSettaDDS.Dds.QoS;
using ROSettaDDS.Msgs.Geometry;
using ROSettaDDS.Msgs.Std;
using ROSettaDDS.Msgs.BuiltinInterfaces;

using RosQuaternion = ROSettaDDS.Msgs.Geometry.Quaternion;
using RosPose = ROSettaDDS.Msgs.Geometry.Pose;
using RosTime = ROSettaDDS.Msgs.BuiltinInterfaces.Time;

namespace SoArmVR.Teleoperation
{
    /// <summary>
    /// テレオペレーションデータを rclsharp (DDS) 経由で ROS 2 トピックに publish する Sink。
    /// </summary>
    public class RosTeleoperationSink : MonoBehaviour, ITeleoperationSink
    {
        DomainParticipant _participant;
        Publisher<PoseStamped> _posePub;
        Publisher<Float64Message> _gripperPub;
        Publisher<BoolMessage> _activePub;

        void Awake()
        {
            InitParticipant();
        }

        void InitParticipant()
        {
            if (_participant != null) return;

            var localIp = LocalNetwork.ResolvePrimaryIPv4();
            var options = new DomainParticipantOptions
            {
                DomainId = 0,
                EntityName = "soarmvr",
                LocalUnicastAddress = localIp,   // null の場合 ROSettaDDS 側で loopback にフォールバック
                MulticastInterface = localIp,
            };
            _participant = new DomainParticipant(options);
            _participant.Start();

            // target_pose は高頻度なので sensor-data 相当の BestEffort。
            _posePub = _participant.CreatePublisher<PoseStamped>(
                "/teleop/target_pose",
                PoseStampedSerializer.Instance,
                ReliabilityQos.BestEffort,
                DurabilityQos.Volatile,
                PoseStamped.DdsTypeName);

            _gripperPub = _participant.CreatePublisher<Float64Message>(
                "/teleop/gripper",
                Float64MessageSerializer.Instance,
                Float64Message.DdsTypeName);

            _activePub = _participant.CreatePublisher<BoolMessage>(
                "/teleop/active",
                BoolMessageSerializer.Instance,
                BoolMessage.DdsTypeName);
        }

        public void OnSessionBegin()
        {
            InitParticipant();
            PublishActive(true);
        }

        public void Push(in TeleoperationSample sample)
        {
            // async void に in パラメータを直接渡せないため値コピーする
            var s = sample;
            PublishPose(s);
            PublishGripper(s.gripper);
        }

        public void OnSessionEnd()
        {
            PublishActive(false);
        }

        void OnDestroy()
        {
            _posePub?.Dispose();
            _gripperPub?.Dispose();
            _activePub?.Dispose();
            _participant?.Dispose();

            _posePub = null;
            _gripperPub = null;
            _activePub = null;
            _participant = null;
        }

        async void PublishPose(TeleoperationSample sample)
        {
            if (_posePub == null) return;

            var now = System.DateTimeOffset.UtcNow;
            var stamp = new RosTime((int)now.ToUnixTimeSeconds(), (uint)(now.Millisecond * 1_000_000));

            var msg = new PoseStamped(
                new Header(stamp, "teleop"),
                new RosPose(
                    new Point(sample.position.x, sample.position.y, sample.position.z),
                    new RosQuaternion(sample.rotation.x, sample.rotation.y, sample.rotation.z, sample.rotation.w)
                )
            );

            try
            {
                await _posePub.PublishAsync(msg);
            }
            catch (System.ObjectDisposedException) { }
        }

        async void PublishGripper(float value)
        {
            if (_gripperPub == null) return;

            try
            {
                await _gripperPub.PublishAsync(new Float64Message(value));
            }
            catch (System.ObjectDisposedException) { }
        }

        async void PublishActive(bool active)
        {
            if (_activePub == null) return;

            try
            {
                await _activePub.PublishAsync(new BoolMessage(active));
            }
            catch (System.ObjectDisposedException) { }
        }
    }
}
