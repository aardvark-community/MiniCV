namespace Aardvark.Reconstruction

open System
open Aardvark.Base
open Aardvark.Reconstruction

module PhotoPair =

    let filterMasks
        (pairs : MapExt<CameraId*CameraId, list<'a*'a>>) 
        (f : V2d) 
        (pp : V2d) 
        (maxNdcAbweichung : float) 
        (getNdc : 'a -> V2d) : MapExt<CameraId*CameraId, list<'a*'a>> = 

        let ff = f.X/f.Y

        //todo f stuff here

        let recoverPoseConfig = 
            RecoverPoseConfig(f.X, pp, 0.999, 0.005)

        pairs |> MapExt.map (fun (lcid,rcid) ms ->
            match ms with
            | [] | [_] -> []
            | v when v.Length < 10 -> []
            | _ ->
                let l = ms |> List.map fst |> List.toArray
                let r = ms |> List.map snd |> List.toArray

                let lcoord = l |> Array.map ( fun ndc -> let ndc = getNdc ndc in V2d(ndc.X, ndc.Y * ff))
                let rcoord = r |> Array.map ( fun ndc -> let ndc = getNdc ndc in V2d(ndc.X, ndc.Y * ff))
            
                let poses, mask = OpenCV.recoverPoses2 recoverPoseConfig lcoord rcoord

                let poses = 
                    poses |> List.collect (fun p ->
                        [
                            p |> CameraPose.scale 1.0 
                            p |> CameraPose.scale -1.0
                        ]
                    )

                let lc = Camera.lookAt (V3d(5.0, 0.0, 0.0)) V3d.OOO V3d.OOI V2d.II

    //            let pose = 
    //                poses |> List.filter ( fun p ->
    //                    let rc = lc |> Camera.transformedView (p |> CameraPose.transformation)
    //                
    //                    let projected = 
    //                        ms |> List.choose ( fun (lf,rf) -> 
    //                            let lray = Camera.unproject1 lc (getNdc lf)
    //                            let rray = Camera.unproject1 rc (getNdc rf)
    //                            
    //                            Ray.intersection [lray; rray]
    //                        ) 
    //                    
    //                    let good = 
    //                        projected.Length > 0 &&
    //                        projected |> List.forall ( fun point -> 
    //                            Vec.dot (point - (rc.location)).Normalized rc.forward >= 0.0
    //                        )
    //
    //                    good
    //                ) |> List.tryHead

                let remain pose =
                    let rc = lc |> Camera.transformedView (pose |> CameraPose.transformation)

                    let remaining =
                        ms |> List.filter ( fun (l,r) ->
                            let l = getNdc l
                            let r = getNdc r

                            let lray = Camera.unproject1 lc l
                            let rray = Camera.unproject1 rc r

                            let intersection = Ray.intersection [lray; rray]

                            match intersection with
                            | None -> false
                            | Some p ->
                                let lp = Camera.project1 lc p
                                let rp = Camera.project1 rc p

                                match lp,rp with
                                | Some lp, Some rp ->
                                    match (l-lp).Length > maxNdcAbweichung, (r-rp).Length > maxNdcAbweichung with
                                    | true, true -> false
                                    | false,false-> true
                                    | _          -> false

                                | _ -> false
                        )

                    remaining

                let allthings = 
                    poses |> List.map remain

                let pose = 
                    let mutable maxItem = Unchecked.defaultof<_>
                    let mutable maxCount = 0
                    for thing in allthings do
                        if thing.Length > maxCount then 
                            maxCount <- thing.Length
                            maxItem <- thing

                    if maxCount < 10 then 
                        None
                    else
                        Some maxItem

                match pose with
                | None -> Log.warn "[PoseFilter] No pose, matches too broken. Camera %A-%A" lcid rcid; []
                | Some pose -> 
                    pose

        )


        