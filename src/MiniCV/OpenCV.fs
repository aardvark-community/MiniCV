namespace Aardvark.Reconstruction

open System
open Aardvark.Base

open System.Collections.Generic
open System.Runtime.InteropServices
open System.Security
open Aardvark.Reconstruction
open Microsoft.FSharp.NativeInterop

#nowarn "9"
#nowarn "51"

[<StructLayout(LayoutKind.Sequential)>]
type RecoverPoseConfig  =
    struct 
            
        val mutable public FocalLength      : double
        val mutable public PrincipalPoint   : V2d
        val mutable public Probability      : double
        val mutable public InlierThreshold  : double
            
        new(f : double, pp : V2d, p : double, t : double) = 
            {
                FocalLength = f
                PrincipalPoint = pp
                Probability = p
                InlierThreshold = t
            }

        static member Default =
            RecoverPoseConfig(1.0, V2d.Zero, 0.999, 0.005)

    end

[<Struct>]
type KeyPoint(index : int, pos : V2d, size : float, angle : float, response : float, octave : int, descriptorDimension : int, descriptors : uint8[]) =
    member x.Index = index
    member x.Position = pos
    member x.Size = size
    member x.Angle = angle
    member x.Response = response
    member x.Octave = int

    member x.Descriptor =
        let fst = int64 descriptorDimension * int64 index
        Vector<uint8>(descriptors, fst, int64 descriptorDimension, 1L)
        
    override x.ToString() =
        sprintf "{ pos = %A; size = %A; angle = %A }" pos size angle

type FeatureResult =
    {
        points                  : KeyPoint[]
        descriptors             : uint8[]
        descriptorDimension     : int
    }


module OpenCV =

    [<StructLayout(LayoutKind.Sequential)>]
    type KeyPoint2d =
        struct
            val mutable public pt : V2f
            val mutable public size : float32
            val mutable public angle : float32
            val mutable public response : float32
            val mutable public octave : int
            val mutable public class_id : int


        end

    [<StructLayout(LayoutKind.Sequential)>]
    type DetectorResult =
        struct
            val mutable public PointCount : int
            val mutable public DescriptorEntries : int
            val mutable public Points : nativeptr<KeyPoint2d>
            val mutable public Descriptors : nativeptr<uint8>
        end

    type DetectorMode =
        | Akaze = 1
        | Orb = 2
        | Brisk = 3


    module Native =

        [<Literal>]
        let lib = @"MiniCVNative"
        
        [<DllImport(lib, EntryPoint = "cvRecoverPose"); SuppressUnmanagedCodeSecurity>]
        extern int cvRecoverPose_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat, V3d& tVec, byte[] ms)
        
        [<DllImport(lib, EntryPoint = "cvRecoverPoses"); SuppressUnmanagedCodeSecurity>]
        extern bool cvRecoverPoses_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat1, M33d& rMat2, V3d& tVec, byte[] ms)
        
        [<DllImport(lib, EntryPoint = "cvDetectFeatures"); SuppressUnmanagedCodeSecurity>]
        extern DetectorResult* cvDetectFeatures_(byte* data, int width, int height, int channels, DetectorMode mode)
        
        [<DllImport(lib, EntryPoint = "cvFreeFeatures"); SuppressUnmanagedCodeSecurity>]
        extern void cvFreeFeatures_(DetectorResult* res)

        [<DllImport(lib, EntryPoint = "cvDoStuff"); SuppressUnmanagedCodeSecurity>]
        extern void cvDoStuff_( nativeint[] imgs, int ct, nativeint[] repr, int rct, nativeint[] oFilenames)

    let private copy (src : nativeptr<'a>) (dst : 'a[]) (cnt : int) =
        let gc = GCHandle.Alloc(dst, GCHandleType.Pinned)
        try
            Marshal.Copy(NativePtr.toNativeInt src, gc.AddrOfPinnedObject(), nativeint sizeof<'a> * nativeint cnt)
        finally
            gc.Free()

    let detectFeatures (mode : DetectorMode) (img : PixImage<byte>) =
        let img = img.ToCanonicalDenseLayout() |> unbox<PixImage<byte>>

        let gc = GCHandle.Alloc(img.Volume.Data, GCHandleType.Pinned)
        let mutable ptr = NativePtr.zero
        try
            ptr <- Native.cvDetectFeatures_(NativePtr.ofNativeInt (gc.AddrOfPinnedObject()), img.Size.X, img.Size.Y, img.ChannelCount, mode)
            let v = NativePtr.read ptr
            
            if v.PointCount = 0 then
                {
                    points                  = [||]
                    descriptors             = [||]
                    descriptorDimension     = 61
                }
            else
                let pts : KeyPoint2d[] = Array.zeroCreate v.PointCount
                let descriptors : uint8[] = Array.zeroCreate v.DescriptorEntries

                copy v.Points pts pts.Length
                copy v.Descriptors descriptors descriptors.Length

                let dim = descriptors.Length / pts.Length

                let points = pts |> Array.mapi (fun i pt -> KeyPoint(i, V2d pt.pt, float pt.size, float pt.angle, float pt.response, pt.octave, dim, descriptors))
                {
                    points                  = points
                    descriptors             = descriptors
                    descriptorDimension     = dim
                }
        finally
            Native.cvFreeFeatures_ ptr
            gc.Free()

    let recoverPose (cfg : RecoverPoseConfig) (a : V2d[]) (b : V2d[]) =
        let mutable m = M33d.Identity
        let mutable t = V3d(100,123,432)
        let mutable cfg = cfg
        let mutable ms = Array.create a.Length 0uy
        let res = Native.cvRecoverPose_(&&cfg, a.Length, a, b, &m, &t, ms)
        (res, m, t, ms)

    let recoverPoses (cfg : RecoverPoseConfig) (a : V2d[]) (b : V2d[]) =
        let mutable m1 = M33d.Identity
        let mutable m2 = M33d.Identity
        let mutable t = V3d(100,123,432)
        let mutable cfg = cfg
        let mutable ms = Array.create a.Length 0uy
        Native.cvRecoverPoses_(&&cfg, a.Length, a, b, &m1, &m2, &t, ms) |> ignore
        (m1, m2, t, ms)

    let recoverPoses2 (cfg : RecoverPoseConfig) (a : V2d[]) (b : V2d[]) =
        let mutable m1 = M33d.Identity
        let mutable m2 = M33d.Identity
        let mutable t = V3d(100,123,432)
        let mutable cfg = cfg
        let mutable ms = Array.create a.Length 0uy
        Native.cvRecoverPoses_(&&cfg, a.Length, a, b, &m1, &m2, &t, ms) |> ignore

        let m1 = m1.Transposed
        let m2 = m2.Transposed

        let possiblePoses =
            if m1 = m2 then
                [CameraPose(0, 1, m1, t, false)]
            else
                [
                    CameraPose(0, 1, m1, t, false)
                    CameraPose(1, 1, m2, t, false)
                ]

        let mask =
            Array.map ((<>) 0uy) ms

        possiblePoses, mask

    open System
    open System.IO
    open System.Text

    let undistortImages (chessboardDir : string) (photoDir : string) =

        let chess = Directory.GetFiles(chessboardDir) 
                        |> Array.map ( fun f -> f.ToLower() ) 
                        |> Array.filter ( fun fn -> Path.GetExtension fn = ".jpg" )
        
        let photo = Directory.GetFiles(photoDir) 
                        |> Array.map ( fun f -> f.ToLower() ) 
                        |> Array.filter ( fun fn -> Path.GetExtension fn = ".jpg" )

        let od =
            let p = Path.combine [photoDir; "undistorted"]
            if p |> Directory.Exists |> not then Directory.CreateDirectory p |> ignore
            p

        let oFiles = 
            photo |> Array.map ( fun f -> Path.combine [od; (Path.GetFileName f)])

//        for c in chess do
//            printfn "%s" c
//
//        for p in photo do
//            printfn "%s" p
//
//        for p in oFiles do
//            printfn "%s" p

        Native.cvDoStuff_(chess |> Array.map Marshal.StringToHGlobalUni, 
                          chess.Length, 
                          photo |> Array.map Marshal.StringToHGlobalUni, 
                          photo.Length, 
                          oFiles |> Array.map Marshal.StringToHGlobalUni)
            