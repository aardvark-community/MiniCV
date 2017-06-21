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
        let res = Native.cvRecoverPoses_(&&cfg, a.Length, a, b, &m1, &m2, &t, ms)
        (res, m1, m2, t, ms)


        
    
    type AngleAxis private() =
        static member RotatePoint(aa : V3d, p : V3d) =
            let theta2 = aa.LengthSquared
            if not (Fun.IsTiny theta2) then
                let theta = sqrt theta2
                let costheta = cos theta
                let sintheta = sin theta
                let thetainverse = 1.0 / theta

                let w = aa * thetainverse

                let wCrossP = Vec.cross w p
                let tmp = (Vec.dot w p) * (1.0 - costheta)


                (p * costheta) + (wCrossP * sintheta) + (w * tmp)

            else
                let wCrossP = Vec.cross aa p
                p + wCrossP

        static member Trafo(aa : V3d) =

            let x = AngleAxis.RotatePoint(aa, V3d.IOO)
            let y = AngleAxis.RotatePoint(aa, V3d.OIO)
            let z = AngleAxis.RotatePoint(aa, V3d.OOI)
            Trafo3d.FromBasis(x, y, z, V3d.Zero)
            


    let toAxisAngle (m : M44d) =
        let m33 = m.UpperLeftM33() 
                
        let rot = m33 |> Rot3d.FromM33d
                
        let mutable axis = V3d.Zero
        let mutable angle = 0.0
        rot.ToAxisAngle(&axis, &angle)

        axis * angle
        
    type Camera3d =
        struct
            val mutable public Position         : V3d
            val mutable public AngleAxis        : V3d

            member x.ProjectWithDepth(p : V3d) =

                let t = - x.Position

                let pt = p + t

                let p' = AngleAxis.RotatePoint( x.AngleAxis, pt )

                let depth = p'.Z

                let proj = p' / depth

                let ndc = proj.XY

                let ndc = V2d(ndc.X, ndc.Y)
                
                ndc, depth

            member x.Project(p : V3d) =
                x.ProjectWithDepth(p) |> fst

            member x.Unproject (pt : V2d) (depthInCameraSpace : float) =
                let t = - x.Position
                let pd = V3d(pt.X, pt.Y, depthInCameraSpace)
                let pr = AngleAxis.RotatePoint(-x.AngleAxis, pd)
                pr - t

            member x.GetRay (pt : V2d) =
                
                let pt = V2d(pt.X, pt.Y)
                let point = x.Unproject pt 1.0
                let dir = (point - x.Position) |> Vec.normalize
                Ray3d(x.Position, dir)

            member x.ViewAndProjTrafo (farPlaneZ : float) =
                //let frustum = { left = -1.0; right = 1.0; top = 1.0; bottom = -1.0; near = 1.0; far = farPlaneZ }

                //translation world -> cam
                let t = -x.Position
                let trans = Trafo3d.Translation(t)

                //rotation world -> cam
                let rot = AngleAxis.Trafo(x.AngleAxis)
                
                assert(rot.Forward.Det > 0.0)

                //trafo world -> cam
                let res = trans * rot
                
                res,
                Trafo3d.Identity

            member x.ViewTrafo =
                x.ViewAndProjTrafo 100.0 |> fst

            member x.ViewProjTrafo (far : float) =
                let (v,p) = x.ViewAndProjTrafo far
                v * p

            //transforms this camera with a trafo given in camera coordinates.
            //example "move this camera 1 unit along its forward direction = translateZ(-1)" //this is negative bc its given in world
            //        "roll this cam to the right = RotateZ( pi/4)" 
            member x.TransformedInCameraSpace (trans : V3d, meToNew : M44d) =
       
                let forward = -V3d.OOI
                let up = V3d.OIO
                let pos = V3d.OOO
                
                
                let forward = meToNew.TransformDir forward
                let up = meToNew.TransformDir up
                let trans = meToNew.TransformDir trans
                

                let forward4 = AngleAxis.RotatePoint(-x.AngleAxis, forward) |> Vec.normalize
                let up4 = AngleAxis.RotatePoint(-x.AngleAxis, up) |> Vec.normalize
                let pos4 = AngleAxis.RotatePoint(-x.AngleAxis, trans) + x.Position

                Camera3d.LookAt(pos4, pos4 + forward4, up4)

            static member LookAt(eye : V3d, center : V3d, sky : V3d) : Camera3d =


                let F = (center - eye)  |> Vec.normalize

                let R = Vec.cross F sky |> Vec.normalize

                let U = Vec.cross R F   |> Vec.normalize
                
                let rot = M44d.FromBasis(R, U, -F, V3d.Zero).Transposed

                

                let aa = (rot |> toAxisAngle)
                

                let c = eye

                Camera3d(c, aa)

            static member Delta(src : Camera3d, dst : Camera3d) =
                let src = src.ViewProjTrafo 100.0
                let dst = dst.ViewProjTrafo 100.0
                src * dst.Inverse

            new(pos, angleAxis) = 
                { Position = pos; AngleAxis = angleAxis}

        end



open MiniCV
module bagd =
    open Aardvark.Base
    open Aardvark.Base.Rendering


    [<EntryPoint>]
    let main argv = 
        
        let c0 = Camera3d.LookAt( V3d(4.0, 5.0, 6.0), V3d.OOO, V3d.OOI )

        for i in 1 .. 10000 do
            let rand = RandomSystem()
            let randomRot = M44d.Rotation( rand.UniformV3dDirection(), rand.UniformDouble() * Constant.PiHalf - Constant.PiQuarter )
            let randomTrans = 3.0 * rand.UniformV3dDirection()

            let c1 = c0.TransformedInCameraSpace(randomTrans, randomRot)

            let test = 
                c0.ViewTrafo.Backward *
                randomRot *
                M44d.Translation(randomTrans) *
                c1.ViewTrafo.Forward

            let test2 = 
                c1.ViewTrafo.Backward *
                M44d.Translation(-randomTrans) *
                randomRot.Inverse *
                c0.ViewTrafo.Forward

                //c1.ViewTrafo.Backward *
                //randomRot.Inverse *
                //M44d.Translation(randomRot.Inverse.TransformDir(randomTrans)) *
                //c0.ViewTrafo.Forward

            printfn "%A" test2

            //c0.ViewTrafo.Forward * randomRot = c1.ViewTrafo




            let pts = Array.init 5000 (fun _ -> rand.UniformV3dDirection())

            let obs0 = pts |> Array.map c0.Project
            let obs1 = pts |> Array.map c1.Project

            let cfg = RecoverPoseConfig.Default

            let ( _, r1a, r2a, t, _ ) = recoverPoses cfg obs0 obs1
            let r1 = M44d.op_Explicit r1a.Transposed
            let r2 = M44d.op_Explicit r2a.Transposed
            let t = 3.0 * t

            let hyptothesis =
                [
                    c0.TransformedInCameraSpace(t, r1)
                    c0.TransformedInCameraSpace(t, r2)
                    c0.TransformedInCameraSpace(-t, r1)
                    c0.TransformedInCameraSpace(-t, r2)
                ]

            let eps = 0.001
            let dist = hyptothesis |> List.map (fun ch -> Vec.length (ch.Position - c1.Position)) |> List.min
            let angle = (acos (abs (Vec.dot t.Normalized randomTrans.Normalized))) * Constant.DegreesPerRadian
            let angle0 = Rot3d.FromM33d((randomRot * r1.Inverse).UpperLeftM33()).GetEulerAngles().Abs * Constant.DegreesPerRadian
            let angle1 = Rot3d.FromM33d((randomRot * r2.Inverse).UpperLeftM33()).GetEulerAngles().Abs * Constant.DegreesPerRadian

            if i % 10 = 0 then printfn "%d" i

            if dist > 0.1 || angle > 3.0 || (angle0.AnyGreater 3.0 && angle1.AnyGreater 3.0) then
                printfn "HATE"
                printfn "angle: %A" angle
                printfn "angle0: %A" angle0
                printfn "angle1: %A" angle1
            
                

        0

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
        