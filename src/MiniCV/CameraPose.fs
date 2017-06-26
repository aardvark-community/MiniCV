namespace Aardvark.Reconstruction

open System
open Aardvark.Base
open Aardvark.Reconstruction

type CameraPose =
    struct
        val mutable public RotationIndex : int
        val mutable public ScaleSign : int
        val mutable public Rotation : M33d
        val mutable public Translation : V3d
        val mutable public IsInverse : bool

        new(ri, ss, r, t, i) = { RotationIndex = ri; ScaleSign = ss; Rotation = r; Translation = t; IsInverse = i }
    end

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module CameraPose =
    let name (pose : CameraPose) =
        let trans = 
            match pose.ScaleSign with
                | 1 -> "t"
                | -1 -> "-t"
                | _ -> "0"

        sprintf "r%d|%s" pose.RotationIndex trans

    let transformation (pose : CameraPose) =
        let m = pose.Rotation

        // M * T (first trans then rotate)
        Trafo3d.FromBasis(m.C0, m.C1, m.C2, m * pose.Translation)

    let scale (f : float) (pose : CameraPose) =
        let s = sign f
        CameraPose(pose.RotationIndex, s * pose.ScaleSign, pose.Rotation, f * pose.Translation, pose.IsInverse)

    let findScaled (inlierThreshold : float) (srcCam : Camera) (worldObservations : list<V3d * V2d>) (pose : CameraPose) =
        // todo: remove outliers
        match worldObservations with
        | [] -> Double.PositiveInfinity, CameraPose()
        | _ ->
            let pose0 = pose |> scale 0.0 |> transformation
            let dstCam0 = srcCam |> Camera.transformedView pose0
        
            let srcView = Camera.viewTrafo srcCam
            let dst0View = Camera.viewTrafo dstCam0
            let dst0Observations =
                worldObservations |> List.map (fun (point, obs) ->
                    dst0View.Forward.TransformPos(point), obs
                )

            let dst0Translation = 
                pose.Rotation * pose.Translation
                    |> srcView.Backward.TransformDir
                    |> dst0View.Forward.TransformDir
                    |> Vec.normalize


            let countInliers (s : float) =
                let finalPose = scale s pose
                let dstCam = Camera.transformedView (transformation finalPose) srcCam

                let mutable count = 0
                for (w,obs) in worldObservations do
                    match Camera.project1 dstCam w with
                        | Some c ->
                            let d = Vec.length (c - obs)
                            if d < inlierThreshold then
                                count <- count + 1
                        | None ->
                                ()
                count
                 

            let mutable bestScale = 0.0
            let mutable bestCount = -1
            let t = dst0Translation
            for (worldPoint, obs) in worldObservations do
                let point = dst0View.Forward.TransformPos(worldPoint)
                let z = (obs * point.Z - point.XY) 
                let n = t.XY - obs * t.Z

                let nt = Fun.IsTiny(n.X, 1E-5) || Fun.IsTiny(n.Y, 1E-5)

                match nt with
                    | true -> ()
                    | _ -> 
                        let s = -z / n

                        let cx = countInliers s.X
                        let cy = countInliers s.Y

                        if cx > bestCount then
                            bestCount <- cx
                            bestScale <- s.X

                        if cy > bestCount then
                            bestCount <- cy
                            bestScale <- s.Y

                        //[s.X; s.Y]

            
            let finalPose = scale bestScale pose
            let dstCam = Camera.transformedView (transformation finalPose) srcCam

            let mutable cnt = 0
            let cost = 1.0 / float (2 + bestCount)
                //worldObservations |> List.sumBy (fun (w,o) -> 
                //    match Camera.project1 dstCam w with
                //        | Some c ->
                //                cnt <- cnt + 1
                //                c - o |> Vec.lengthSquared
                //        | None ->
                //                0.0
                //)

            //let avgCost = sqrt (cost / float cnt)

            cost, finalPose


            //let scales =
            //    let t = dst0Translation
            //    worldObservations |> List.collect (fun (worldPoint, obs) ->
            //        let point = dst0View.Forward.TransformPos(worldPoint)
            //        // project(point + s * t) = obs
                
            //        // (point.xy + s * t.xy) / (point.z + s * t.z) = obs
            //        // point.xy + s * t.xy  = obs * (point.z + s * t.z)
            //        // point.xy + s * t.xy  = obs * point.z + s * obs * t.z
            //        // s * t.xy - s * obs * t.z  = obs * point.z - point.xy
            //        // s * (t.xy - obs * t.z) = obs * point.z - point.xy
            //        // s = (obs * point.z - point.xy) / (t.xy - obs * t.z)
            //        let z = (obs * point.Z - point.XY) 
            //        let n = t.XY - obs * t.Z

            //        let nt = Fun.IsTiny(n.X, 1E-5) || Fun.IsTiny(n.Y, 1E-5)

            //        match nt with
            //            | true -> []
            //            | _ -> 
            //                let s = -z / n

            //                [s.X; s.Y]


            //    )
            
            //match scales with
            //| [] -> Double.PositiveInfinity, CameraPose()
            //| _ ->
            //    let scaleRange = Range1d scales
            //    let s = List.average scales

            //    let finalPose = scale s pose
            //    let dstCam = Camera.transformedView (transformation finalPose) srcCam

            //    let mutable cnt = 0
            //    let cost = 
            //        worldObservations |> List.sumBy (fun (w,o) -> 
            //            match Camera.project1 dstCam w with
            //                | Some c ->
            //                        cnt <- cnt + 1
            //                        c - o |> Vec.lengthSquared
            //                | None ->
            //                        0.0
            //        )

            //    let avgCost = sqrt (cost / float cnt)

            //    avgCost, finalPose

    let inverse (pose : CameraPose) =
        // qi = R * (pi + t)
        // => pi = R^-1 * qi - t
        // => pi = R^-1 * (qi - R * t)

        // => R' = R^-1
        // => t' = - R * t

        let R = pose.Rotation
        let t = pose.Translation

        let R' = R.Transposed // rotation inverse
        let t' = -(R * t)
        CameraPose(pose.RotationIndex, pose.ScaleSign, R', t', not pose.IsInverse)
