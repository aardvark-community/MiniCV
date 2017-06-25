namespace Aardvark.Reconstruction

open System
open Aardvark.Base

open System.Collections.Generic
open System.Runtime.InteropServices
open System.Security
open Aardvark.Reconstruction

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
            RecoverPoseConfig(1.0, V2d.Zero, 0.999, 0.01)

    end

module OpenCV =

    module Native =

        [<Literal>]
        let lib = @"MiniCVNative"
        
        [<DllImport(lib, EntryPoint = "cvRecoverPose"); SuppressUnmanagedCodeSecurity>]
        extern int cvRecoverPose_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat, V3d& tVec, byte[] ms)
        
        [<DllImport(lib, EntryPoint = "cvRecoverPoses"); SuppressUnmanagedCodeSecurity>]
        extern bool cvRecoverPoses_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat1, M33d& rMat2, V3d& tVec, byte[] ms)

        [<DllImport(lib, EntryPoint = "cvDoStuff"); SuppressUnmanagedCodeSecurity>]
        extern void cvDoStuff_( string[] imgs, int ct, string[] repr, int rct, string[] oFilenames)


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

        Native.cvDoStuff_(chess, 
                          chess.Length, 
                          photo, 
                          photo.Length, 
                          oFiles)
            