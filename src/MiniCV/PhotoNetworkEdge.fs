namespace Aardvark.Reconstruction

open System
open Aardvark.Base
open Aardvark.Reconstruction

type PhotoNetworkEdge =
    {
        left                : CameraId
        right               : CameraId
        leftObservations    : MapExt<TrackId, V2d>
        rightObservations   : MapExt<TrackId, V2d>
        inverse             : bool

        leftToRight         : list<CameraPose>
        rightToLeft         : list<CameraPose>

        trackCount          : int
        tracks              : Set<TrackId>
    }

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module PhotoNetworkEdge =

    let empty (lid : CameraId) (rid : CameraId) =
        {
            left                = lid
            right               = rid
            leftObservations    = MapExt.empty
            rightObservations   = MapExt.empty
            inverse             = false

            leftToRight         = []
            rightToLeft         = []

            trackCount          = 0
            tracks              = Set.empty
        }

    let create (lid : CameraId) (lObs : MapExt<TrackId, V2d>) (rid : CameraId) (rObs : MapExt<TrackId, V2d>) =
        let mutable bothObs = MapExt.intersect lObs rObs
        if bothObs.Count < 5 then
            empty lid rid
        else
            let trackIds, observations = bothObs |> MapExt.toArray |> Array.unzip
            let l, r = Array.unzip observations

            let poses, mask = OpenCV.recoverPoses2 RecoverPoseConfig.Default l r

            let mutable trackCount = 0
            let mutable tracks = Set.empty
            for i in 0 .. mask.Length - 1 do
                if mask.[i] then
                    tracks <- Set.add trackIds.[i] tracks
                    trackCount <- trackCount + 1
                else
                    bothObs <- MapExt.remove trackIds.[i] bothObs

            {
                left = lid
                right = rid
                inverse = false
                leftObservations = MapExt.map (fun _ -> fst) bothObs
                rightObservations = MapExt.map (fun _ -> snd) bothObs

                leftToRight = poses
                rightToLeft = poses |> List.map CameraPose.inverse

                trackCount = trackCount
                tracks = tracks
            }

    let inverse (edge : PhotoNetworkEdge) =
        {
            left                = edge.right
            right               = edge.left
            inverse             = not edge.inverse
            leftObservations    = edge.rightObservations
            rightObservations   = edge.leftObservations

            leftToRight         = edge.rightToLeft
            rightToLeft         = edge.leftToRight

            trackCount          = edge.trackCount
            tracks              = edge.tracks
        }
