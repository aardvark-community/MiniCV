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
        
        [<DllImport(lib, EntryPoint = "cvRecoverPoses"); SuppressUnmanagedCodeSecurity>]
        extern void cvRecoverPoses_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat1, M33d& rMat2, V3d& tVec, byte[] ms)

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
        printfn "%A" System.Environment.CurrentDirectory
        let res = Native.cvRecoverPoses_(&&cfg, a.Length, a, b, &m1, &m2, &t, ms)
        (res, m1, m2, t, ms)


        
    
    //type AngleAxis private() =
    //    static member RotatePoint(aa : V3d, p : V3d) =
    //        let theta2 = aa.LengthSquared
    //        if not (Fun.IsTiny theta2) then
    //            let theta = sqrt theta2
    //            let costheta = cos theta
    //            let sintheta = sin theta
    //            let thetainverse = 1.0 / theta

    //            let w = aa * thetainverse

    //            let wCrossP = Vec.cross w p
    //            let tmp = (Vec.dot w p) * (1.0 - costheta)


    //            (p * costheta) + (wCrossP * sintheta) + (w * tmp)

    //        else
    //            let wCrossP = Vec.cross aa p
    //            p + wCrossP

    //    static member Trafo(aa : V3d) =

    //        let x = AngleAxis.RotatePoint(aa, V3d.IOO)
    //        let y = AngleAxis.RotatePoint(aa, V3d.OIO)
    //        let z = AngleAxis.RotatePoint(aa, V3d.OOI)
    //        Trafo3d.FromBasis(x, y, z, V3d.Zero)
            
    //type Camera3d =
    //    struct
    //        val mutable public Position         : V3d
    //        val mutable public AngleAxis        : V3d

    //        member x.ProjectWithDepth(p : V3d) =
    //            let view = AngleAxis.RotatePoint(x.AngleAxis, p - x.Position)
    //            let ndc = view.XY / view.Z

    //            ndc, view.Z

    //        member x.Project(p : V3d) =
    //            let view = AngleAxis.RotatePoint(x.AngleAxis, p - x.Position)
    //            let ndc = view.XY / view.Z

    //            ndc

    //            //V2d(0.5 * ndc.X - 0.5, 0.5 - 0.5 * ndc.Y)

    //            //ndc * 0.5 + V2d.II

    //        member x.Transformed (t : Trafo3d) =
    //            let up = AngleAxis.RotatePoint(-x.AngleAxis, V3d.OIO)
    //            let fw = AngleAxis.RotatePoint(-x.AngleAxis, -V3d.OOI)
    //            let p =  x.Position         |> t.Forward.TransformPosProj
    //            let pu = x.Position + up    |> t.Forward.TransformPosProj
    //            let pf = x.Position + fw    |> t.Forward.TransformPosProj

    //            let u = pu - p
    //            let s = Vec.length u

    //            let mutable res = Camera3d.LookAt(p, pf, (pf - p).Length / s, u / s)
    //            res

    //        member x.Unproject (pt : V2d) (depth : float) =
    //            let ndc = V2d(pt.X, pt.Y)
    //            let dir = AngleAxis.RotatePoint(-x.AngleAxis, V3d(ndc, 1.0) * depth)
    //            x.Position + dir

    //        member x.GetRay (pt : V2d) =
    //            let point = x.Unproject pt 1.0
    //            let dir = (point - x.Position) |> Vec.normalize
    //            Ray3d(x.Position, dir)

    //        static member LookAt(eye : V3d, center : V3d, f : float, sky : V3d) : Camera3d =
    //            let forward = Vec.normalize (center - eye)
    //            let left = Vec.cross sky forward |> Vec.normalize
    //            let up = Vec.cross forward left |> Vec.normalize

    //            let rot = M44d.FromBasis(-left, up, -forward, V3d.Zero).UpperLeftM33() |> Rot3d.FromM33d
    //            let mutable axis = V3d.Zero
    //            let mutable angle = 0.0
    //            rot.ToAxisAngle(&axis, &angle)
    //            let aa = axis * -angle
    //            let res = Camera3d(eye, aa)

    //            let test = res.Project center
    //            res
                
    //        new(pos, angleAxis) = 
    //            { Position = pos; AngleAxis = angleAxis}
    //    end






    open Aardvark.Base
    open Aardvark.Base.Rendering


    //[<EntryPoint>]
    let main argv = 

        //let aa = (-V3d.OOI) * Constant.PiHalf

        //let rot = Rot3d((-V3d.OOI), Constant.PiHalf)

        //let p = V3d(1235.346,-678.234,-12412.678)

        //let rotated2 = 
        //    rot.TransformPos(p)
        
        //let rotated1 =
        //    AngleAxis.RotatePoint(aa, p)

        //printfn "AngleAxis: %A" rotated1
        //printfn "Rot3d:     %A" rotated2

        //let trafo = AngleAxis.Trafo aa

        //printfn "%A" trafo









        //let fufu = 
        //    [|
        //        V3d(-0.2, 0.5, 0.91)
        //        V3d(-0.4, -0.4, 0.83)
        //        V3d(-0.2, 0.6, 0.94)
        //        V3d(-0.5, -0.3, 0.85)
        //        V3d(-0.7, 0.2, 0.96)
        //        V3d(-0.1, -0.1, 0.87)
        //        V3d(0.6, 0.2, 0.98)
        //        V3d(0.7, -0.1, 0.89)
        //        V3d(0.3, 0.4, 0.94)
        //        V3d(0.4, -0.5, 0.81)
        //        V3d(0.4, 0.6, 0.96)
        //        V3d(0.3, -0.7, 0.93)
        //    |]
        
        //let up = M44d.RotationX(Constant.RadiansPerDegree * 30.0).TransformDir V3d.OOI

        //let cam1 = Camera3d.LookAt((V3d(1.0,1.0,1.0)),V3d.Zero,1.0,V3d.OOI)
        //let cam2 = Camera3d.LookAt((V3d(-4.0,1.0,1.0)),V3d.Zero,1.0,V3d.OOI)
        
        //let x = AngleAxis.Trafo(-cam1.AngleAxis) * Trafo3d.Translation(cam1.Position-cam2.Position) * AngleAxis.Trafo cam2.AngleAxis
        //// 0.5 - 0.5 * x
        //// 1.0 - 2*x
        //let fufu1 = fufu |> Array.map cam1.Project
        //let fufu2 = fufu |> Array.map cam2.Project

        //printfn "fufu1:%A fufu2:%A" fufu1.Length fufu2.Length

        //let (inliers, R1, R2, t, mask) = recoverPoses (RecoverPoseConfig(1.0, V2d.Zero, 0.99, 0.01)) fufu1 fufu2

        //printfn "t=%A" t
        //printfn "R1=\n%A\n" R1
        //printfn "R2=\n%A\n" R2

        //let real = (Trafo3d.FromBasis(R.C0, R.C1, R.C2, t).Forward).Elements |> Seq.map (fun e -> Math.Round(e,3)) |> Seq.toArray |> M44d
        //printfn "reference=%A\n" (x.Forward.Elements |> Seq.map (fun e -> Math.Round(e,3)) |> Seq.toArray |> M44d)
        //printfn "real=     %A\n" real

        
        
        0

        //let rand = RandomSystem()

        //let mutable fuck = Unchecked.defaultof<_>

        //let (inliers, R, t, mask) = recoverPose (RecoverPoseConfig(1.0, V2d.Zero, 0.99, 0.01)) a b

        //async {
        //    let pts = Array.init 10 (fun _ -> rand.UniformV3d(Box3d(-(V3d(0.1,0.1,0.1)),(V3d(0.1,0.1,0.1)))) ) 
        //    let pts2 = 
        //        Array.init 10 (fun _ -> rand.UniformV3d(Box3d(-(V3d(0.1,0.1,0.1)),(V3d(0.1,0.1,0.1)))) ) 
        //        //pts |> Array.map ((+) V3d.IOO)
    
        //    let a = pts |> Array.map ( fun x -> x.XY / x.Z )
        //    let b = pts2 |> Array.map ( fun x -> x.XY / x.Z )

        //    let (inliers, R, t, mask) = recoverPose (RecoverPoseConfig(1.0, V2d.Zero, 0.99999, 0.0001)) a b

        //    printfn "%A" mask
        //    fuck <- mask
        //} |> Async.Start

        //while true do
        //    printfn "%A" fuck

        //0
        