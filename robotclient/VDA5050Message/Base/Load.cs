using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Load: VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Load_GetLoadId(IntPtr load);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Load_GetLoadType(IntPtr load);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Load_GetLoadPosition(IntPtr load);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint Load_GetWeight(IntPtr load);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Load_GetBoundingBoxReference(IntPtr load);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Load_GetLoadDimensions(IntPtr load);

        public string? LoadId { get; set; }
        public string? LoadType { get; set; }
        public string? LoadPosition { get; set; }
        public BoundingBoxReference? BoundingBoxReference { get; set; }
        public LoadDimensions? LoadDimensions { get; set; }
        public double? Weight { get; set; }

        public override void CreateWrapper()
        {
            // Load is a nested "get-only" structure (read from State wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            LoadId = VDA5050MessageBase.PtrToString(Load_GetLoadId(prt));
            LoadType = VDA5050MessageBase.PtrToString(Load_GetLoadType(prt));
            LoadPosition = VDA5050MessageBase.PtrToString(Load_GetLoadPosition(prt));
            Weight = Load_GetWeight(prt);

            var bbPtr = Load_GetBoundingBoxReference(prt);
            if (bbPtr != IntPtr.Zero)
            {
                BoundingBoxReference ??= new BoundingBoxReference();
                BoundingBoxReference.GetDataWrapper(bbPtr);
            }

            var dimPtr = Load_GetLoadDimensions(prt);
            if (dimPtr != IntPtr.Zero)
            {
                LoadDimensions ??= new LoadDimensions();
                LoadDimensions.GetDataWrapper(dimPtr);
            }
        }
    }

    public class BoundingBoxReference: VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double BoundingBoxReference_GetX(IntPtr bb);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double BoundingBoxReference_GetY(IntPtr bb);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double BoundingBoxReference_GetZ(IntPtr bb);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double BoundingBoxReference_GetTheta(IntPtr bb);

        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double? Theta { get; set; }

        public override void CreateWrapper()
        {
            // BoundingBoxReference is a nested "get-only" structure (read from Load wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            X = BoundingBoxReference_GetX(prt);
            Y = BoundingBoxReference_GetY(prt);
            Z = BoundingBoxReference_GetZ(prt);
            Theta = BoundingBoxReference_GetTheta(prt);
        }
    }

    public class LoadDimensions: VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double LoadDimensions_GetLength(IntPtr dim);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double LoadDimensions_GetWidth(IntPtr dim);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double LoadDimensions_GetHeight(IntPtr dim);

        public double Length { get; set; }
        public double Width { get; set; }
        public double? Height { get; set; }

        public override void CreateWrapper()
        {
            // LoadDimensions is a nested "get-only" structure (read from Load wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            Length = LoadDimensions_GetLength(prt);
            Width = LoadDimensions_GetWidth(prt);
            Height = LoadDimensions_GetHeight(prt);
        }
    }
}
