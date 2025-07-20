// (c) Copyright VinforLab Team. All rights reserved.

using UnityEngine;

namespace VinforlabTeam.FurClimbingSystem
{
    public class FCSTypes
    {
        public enum NumberComparison
        {
            EQUAL,
            LESS_THAN,
            GREATER_THAN
        }

        public enum Axis
        {
            UP,
            DOWN,
            RIGHT,
            LEFT,
            FORWARD,
            BACKWARD
        }

        public enum UpdateMode
        {
            FIXED_UPDATE,
            UPDATE,
            LATE_UPDATE,
            IENUMERATOR
        }

        public enum Pivot
        {
            X, Y, Z
        }

        public static Vector3 AxisEnumToLocalVector3(Transform t, Axis axis)
        {
            if (axis == FCSTypes.Axis.DOWN)
            {
                return -t.up;
            }

            if (axis == FCSTypes.Axis.UP)
            {
                return t.up;
            }

            if (axis == FCSTypes.Axis.RIGHT)
            {
                return t.right;
            }

            if (axis == FCSTypes.Axis.LEFT)
            {
                return -t.right;
            }

            if (axis == FCSTypes.Axis.FORWARD)
            {
                return t.forward;
            }

            if (axis == FCSTypes.Axis.BACKWARD)
            {
                return -t.forward;
            }

            return t.up;
        }

        public static Vector3 AxisEnumToWorldVector3(Axis axis)
        {
            if (axis == FCSTypes.Axis.DOWN)
            {
                return -Vector3.up;
            }

            if (axis == FCSTypes.Axis.UP)
            {
                return Vector3.up;
            }

            if (axis == FCSTypes.Axis.RIGHT)
            {
                return Vector3.right;
            }

            if (axis == FCSTypes.Axis.LEFT)
            {
                return -Vector3.right;
            }

            if (axis == FCSTypes.Axis.FORWARD)
            {
                return Vector3.forward;
            }

            if (axis == FCSTypes.Axis.BACKWARD)
            {
                return -Vector3.forward;
            }

            return Vector3.up;
        }
    }
}