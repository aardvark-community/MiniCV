namespace Aardvark.Base

open System
open Aardvark.Base

open System.Runtime.InteropServices
open System.Security

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
                RecoverPoseConfig(1.0, V2d.Zero, 0.99999, 0.0001)

    end

module MiniCV =
   

    module Native =
        [<Literal>]
        let lib = "MiniCVNative"
        
        [<DllImport(lib, EntryPoint = "cvRecoverPose"); SuppressUnmanagedCodeSecurity>]
        extern int cvRecoverPose_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat, V3d& tVec, byte[] ms)

    let recoverPose (cfg : RecoverPoseConfig) (a : V2d[]) (b : V2d[]) =
        let mutable m = M33d.Identity
        let mutable t = V3d(100,123,432)
        let mutable cfg = cfg
        let mutable ms = Array.create a.Length 0uy
        let res = Native.cvRecoverPose_(&&cfg, a.Length, a, b, &m, &t, ms)
        (res, m, t, ms)


    //[<EntryPoint>]
    //let main argv = 

    //    let rand = RandomSystem()

    //    let mutable fuck = Unchecked.defaultof<_>

    //    async {
    //        let pts = Array.init 10 (fun _ -> rand.UniformV3d(Box3d(-(V3d(0.1,0.1,0.1)),(V3d(0.1,0.1,0.1)))) ) 
    //        let pts2 = 
    //            Array.init 10 (fun _ -> rand.UniformV3d(Box3d(-(V3d(0.1,0.1,0.1)),(V3d(0.1,0.1,0.1)))) ) 
    //            //pts |> Array.map ((+) V3d.IOO)
    
    //        let a = pts |> Array.map ( fun x -> x.XY / x.Z )
    //        let b = pts2 |> Array.map ( fun x -> x.XY / x.Z )

    //        let (inliers, R, t, mask) = recoverPose (RecoverPoseConfig(1.0, V2d.Zero, 0.99999, 0.0001)) a b

    //        printfn "%A" mask
    //        fuck <- mask
    //    } |> Async.Start

    //    while true do
    //        printfn "%A" fuck

    //    0
        