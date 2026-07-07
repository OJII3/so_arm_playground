using System;
using UnityEngine;

namespace SoArmVR.Kinematics
{
    public static class UrdfStlParser
    {
        public static Mesh ParseBinary(byte[] data)
        {
            if (data == null || data.Length < 84)
            {
                Debug.LogWarning("STL data too short");
                return null;
            }

            int triCount = BitConverter.ToInt32(data, 80);
            int expectedLen = 84 + triCount * 50;
            if (data.Length < expectedLen)
            {
                Debug.LogWarning($"STL size mismatch: got {data.Length}, expected {expectedLen}");
                return null;
            }

            var vertices = new Vector3[triCount * 3];
            var normals = new Vector3[triCount * 3];
            var triangles = new int[triCount * 3];

            for (int i = 0; i < triCount; i++)
            {
                int offset = 84 + i * 50;
                float nx = BitConverter.ToSingle(data, offset);
                float ny = BitConverter.ToSingle(data, offset + 4);
                float nz = BitConverter.ToSingle(data, offset + 8);
                var n = new Vector3(nx, ny, nz);

                // STL binary stores vertices in arbitrary winding order;
                // Unity expects counter-clockwise.  SWAP 1→2 to reverse CW→CCW.
                int t0 = i * 3;
                for (int j = 0; j < 3; j++)
                {
                    int vi = t0 + j;
                    int vo = offset + 12 + j * 12;
                    vertices[vi] = new Vector3(
                        BitConverter.ToSingle(data, vo),
                        BitConverter.ToSingle(data, vo + 4),
                        BitConverter.ToSingle(data, vo + 8)
                    );
                    normals[vi] = n;
                }
                triangles[t0] = t0;
                triangles[t0 + 1] = t0 + 2;   // swap
                triangles[t0 + 2] = t0 + 1;
            }

            var mesh = new Mesh();
            mesh.indexFormat = triCount > 21845
                ? UnityEngine.Rendering.IndexFormat.UInt32
                : UnityEngine.Rendering.IndexFormat.UInt16;
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.normals = normals;
            mesh.RecalculateBounds();

            return mesh;
        }
    }
}
