using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Trajectory : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Trajectory_Create();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Trajectory_Destroy(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Trajectory_SetDegree(IntPtr wrapper, int degree);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Trajectory_SetKnotVector(IntPtr wrapper, double[] data, int length);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Trajectory_ClearControlPoints(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Trajectory_AddControlPoint(IntPtr wrapper, IntPtr controlPoint);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int Trajectory_GetDegree(IntPtr trajectory);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int Trajectory_GetKnotVectorCount(IntPtr trajectory);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double Trajectory_GetKnotVectorAt(IntPtr trajectory, int index);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int Trajectory_GetControlPointsCount(IntPtr trajectory);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Trajectory_GetControlPointAt(IntPtr trajectory, int index);

        public int Degree { get; set; }

        public List<double> KnotVector { get; set; }

        public List<ControlPoint> ControlPoints { get; set; }

        public override void CreateWrapper()
        {
            if (_wrapperPtr.HasValue)
            {
                Trajectory_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }

            var prt = Trajectory_Create();
            _wrapperPtr = prt;

            Trajectory_SetDegree(prt, Degree);

            if (KnotVector != null)
            {
                var data = KnotVector.ToArray();
                Trajectory_SetKnotVector(prt, data, data.Length);
            }

            Trajectory_ClearControlPoints(prt);
            if (ControlPoints != null)
            {
                foreach (var controlPoint in ControlPoints)
                {
                    controlPoint.CreateWrapper();
                    if (controlPoint._wrapperPtr.HasValue)
                    {
                        Trajectory_AddControlPoint(prt, controlPoint._wrapperPtr.Value);
                    }
                }
            }
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            Degree = Trajectory_GetDegree(prt);

            var knotCount = Trajectory_GetKnotVectorCount(prt);
            KnotVector ??= new List<double>();
            KnotVector.Clear();
            for (var i = 0; i < knotCount; i++)
            {
                KnotVector.Add(Trajectory_GetKnotVectorAt(prt, i));
            }

            var cpCount = Trajectory_GetControlPointsCount(prt);
            ControlPoints ??= new List<ControlPoint>();
            ControlPoints.Clear();
            for (var i = 0; i < cpCount; i++)
            {
                var cpPtr = Trajectory_GetControlPointAt(prt, i);
                if (cpPtr == IntPtr.Zero)
                {
                    continue;
                }
                var cp = new ControlPoint();
                cp.GetDataWrapper(cpPtr);
                ControlPoints.Add(cp);
            }
        }

        ~Trajectory()
        {
            if (_wrapperPtr.HasValue)
            {
                Trajectory_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }
        }
    }
}
