using UnityEngine;
using ROSettaDDS.Rcl;
using ROSettaDDS.Dds.QoS;
using ROSettaDDS.Msgs.Geometry;
using ROSettaDDS.Msgs.Std;
using ROSettaDDS.Msgs.BuiltinInterfaces;
using ROSettaDDS.Msgs.TeleopIk;

using RosQuaternion = ROSettaDDS.Msgs.Geometry.Quaternion;
using RosPose = ROSettaDDS.Msgs.Geometry.Pose;
using RosTime = ROSettaDDS.Msgs.BuiltinInterfaces.Time;
using RosTargetPoseWithInput = ROSettaDDS.Msgs.TeleopIk.TargetPoseWithInput;

namespace SoArmVR.Teleoperation
{
    /// <summary>
    /// テレオペレーションデータを ROSettaDDS (DDS) 経由で ROS 2 トピックに publish する Sink。
    /// rcl 的な <see cref="Context"/> + <see cref="Node"/> 2 層 API を使う。
    /// </summary>
    public class RosTeleoperationSink : MonoBehaviour, ITeleoperationSink
    {
        Context _context;
        Node _node;
        Publisher<RosTargetPoseWithInput> _targetPub;
        Publisher<Float64Message> _gripperPub;
        Publisher<BoolMessage> _activePub;
        Publisher<ResetCommand> _resetPub;

        void Awake()
        {
            InitParticipant();
        }

        void InitParticipant()
        {
            if (_context != null) return;

            // LocalUnicastAddress 未指定 (既定) で ROSettaDDS が全 NIC を自動列挙して広告する。
            _context = new Context(new ContextOptions
            {
                DomainId = 0,
                EntityName = "soarmvr",
            });
            _context.Start();

            _node = new Node(_context, "soarmvr");

            // target は高頻度なので sensor-data 相当の BestEffort。
            _targetPub = _node.CreatePublisher<RosTargetPoseWithInput>(
                "/teleop/target",
                TargetPoseWithInputSerializer.Instance,
                ReliabilityQos.BestEffort,
                DurabilityQos.Volatile,
                RosTargetPoseWithInput.DdsTypeName);

            _gripperPub = _node.CreatePublisher<Float64Message>(
                "/teleop/gripper",
                Float64MessageSerializer.Instance,
                Float64Message.DdsTypeName);

            _activePub = _node.CreatePublisher<BoolMessage>(
                "/teleop/active",
                BoolMessageSerializer.Instance,
                BoolMessage.DdsTypeName);

            _resetPub = _node.CreatePublisher<ResetCommand>(
                "/teleop/reset",
                ResetCommandSerializer.Instance,
                ReliabilityQos.Reliable,
                DurabilityQos.Volatile,
                ResetCommand.DdsTypeName);
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
            PublishTarget(s);
            PublishGripper(s.gripper);
        }

        public void OnSessionEnd()
        {
            PublishActive(false);
        }

        void OnDestroy()
        {
            _targetPub?.Dispose();
            _gripperPub?.Dispose();
            _activePub?.Dispose();
            _resetPub?.Dispose();
            _node?.Dispose();
            _context?.Dispose();

            _targetPub = null;
            _gripperPub = null;
            _activePub = null;
            _resetPub = null;
            _node = null;
            _context = null;
        }

        async void PublishTarget(TeleoperationSample sample)
        {
            if (_targetPub == null) return;

            var now = System.DateTimeOffset.UtcNow;
            var stamp = new RosTime((int)now.ToUnixTimeSeconds(), (uint)(now.Millisecond * 1_000_000));

            var msg = new RosTargetPoseWithInput(
                new Header(stamp, "teleop"),
                new RosPose(
                    new Point(sample.position.x, sample.position.y, sample.position.z),
                    new RosQuaternion(sample.rotation.x, sample.rotation.y, sample.rotation.z, sample.rotation.w)
                ),
                sample.stick.x,
                sample.stick.y,
                sample.ikActive
            );

            try
            {
                await _targetPub.PublishAsync(msg);
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

        public void PublishReset()
        {
            _ = PublishResetAsync();
        }

        async System.Threading.Tasks.Task PublishResetAsync()
        {
            if (_resetPub == null) return;
            var now = System.DateTimeOffset.UtcNow;
            var stamp = new RosTime((int)now.ToUnixTimeSeconds(), (uint)(now.Millisecond * 1_000_000));
            var msg = new ResetCommand(
                new Header(stamp, "teleop_reset"),
                new float[] { float.NaN, float.NaN, float.NaN, float.NaN, float.NaN, float.NaN },
                0.0f
            );
            try
            {
                await _resetPub.PublishAsync(msg);
            }
            catch (System.ObjectDisposedException) { }
        }
    }
}
