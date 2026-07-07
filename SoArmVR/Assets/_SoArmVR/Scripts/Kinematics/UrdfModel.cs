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
}
