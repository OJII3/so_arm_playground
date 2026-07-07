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

            // Collect joints
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

                if (err.magnitude < (float)Tolerance)
                {
                    ClampJoints(model, qOut);
                    return true;
                }

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

                var jjt = J * J.Transpose();
                jjt[0, 0] += Damping;
                jjt[1, 1] += Damping;
                jjt[2, 2] += Damping;

                var jjtInv = jjt.Inverse();
                var errV3 = new Vector3(err.x, err.y, err.z);
                var dqPosV3 = J.Transpose() * (jjtInv * errV3);

                for (int j = 0; j < 3; j++)
                {
                    qOut[j] += dqPosV3[j] * IkDt;
                }

                ClampJoints(model, qOut);
            }

            return false;
        }

        static void ClampJoints(UrdfModel model, double[] q)
        {
            for (int ji = 0; ji < model.Joints.Count; ji++)
            {
                var j = model.Joints[ji];
                int idx = System.Array.IndexOf(new[] { "1", "2", "3", "4", "5", "6" }, j.name);
                if (idx < 0 || idx >= q.Length) continue;
                q[idx] = System.Math.Clamp(q[idx], j.limit.lower, j.limit.upper);
            }
        }

        struct Matrix3x3
        {
            public double[,] m;
            public Matrix3x3(bool init) => m = new double[3, 3];
            public double this[int r, int c]
            {
                get => m?[r, c] ?? 0;
                set { if (m == null) m = new double[3, 3]; m[r, c] = value; }
            }

            public Matrix3x3 Transpose()
            {
                var r = new Matrix3x3(true);
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        r[j, i] = this[i, j];
                return r;
            }

            public static Matrix3x3 operator *(Matrix3x3 a, Matrix3x3 b)
            {
                var r = new Matrix3x3(true);
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
                if (System.Math.Abs(det) < 1e-15) return new Matrix3x3(true);

                double invDet = 1.0 / det;
                var r = new Matrix3x3(true);
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
}
