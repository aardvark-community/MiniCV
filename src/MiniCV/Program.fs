﻿namespace Aardvark.Base

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


type Camera =
    {
        location    : V3d
        forward     : V3d
        up          : V3d
        right       : V3d
    }

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module Camera =
    let viewTrafo (c : Camera) =
        let z = -c.forward
        let x = c.right
        let y = c.up
        let p = c.location
        Trafo3d.FromBasis(x,y,z,p).Inverse
    
    let transformed (t : Trafo3d) (c : Camera) =
        let fw = t.Forward
        {
            location = fw.TransformPos(c.location)
            forward = fw.TransformDir(c.forward)        |> Vec.normalize
            up = fw.TransformDir(c.up)                  |> Vec.normalize
            right = fw.TransformDir(c.right)            |> Vec.normalize
        }

    let transformedView (t : Trafo3d) (c : Camera) =
        let t = t.Forward
        let toWorld = M44d.FromBasis(c.right, c.up, -c.forward, c.location)
        let toRotWorld = toWorld * t
        {
            location    = toRotWorld.TransformPos(V3d.Zero)
            forward     = toRotWorld.TransformDir(-V3d.OOI) |> Vec.normalize
            up          = toRotWorld.TransformDir(V3d.OIO) |> Vec.normalize
            right       = toRotWorld.TransformDir(V3d.IOO) |> Vec.normalize
        }
       
    let project1 (c : Camera) (pt : V3d) =
        let o = pt - c.location
        let pc = 
            V3d(
                Vec.dot o c.right,
                Vec.dot o c.up,
                Vec.dot o -c.forward
            )

        pc.XY / pc.Z

    let unproject1 (c : Camera) (pt : V2d) =
        let pc = V3d(pt, 1.0)
        let direction =
            pc.X * c.right +
            pc.Y * c.up +
            pc.Z * -c.forward

        Ray3d(c.location, Vec.normalize direction)

    let lookAt (eye : V3d) (center : V3d) (sky : V3d) =
        let fw = center - eye       |> Vec.normalize
        let r = Vec.cross fw sky    |> Vec.normalize
        let u = Vec.cross r fw      |> Vec.normalize

        { 
            location = eye
            forward = fw
            up = u
            right = r
        }

    let angles (l : Camera) (r : Camera) =
        let af = acos (Vec.dot l.forward r.forward |> clamp -1.0 1.0) |> abs
        let ar = acos (Vec.dot l.right r.right |> clamp -1.0 1.0) |> abs
        let au = acos (Vec.dot l.up r.up |> clamp -1.0 1.0) |> abs
        V3d(ar, au, af)

    let distance (l : Camera) (r : Camera) =    
        Vec.length (l.location - r.location)

    let approxEqual (angleTol : float) (spatialTol : float) (l : Camera) (r : Camera) =
        
        let dl = Vec.length (l.location - r.location) |> abs
        let af = acos (Vec.dot l.forward r.forward |> clamp -1.0 1.0) |> abs
        let ar = acos (Vec.dot l.right r.right |> clamp -1.0 1.0) |> abs
        let au = acos (Vec.dot l.up r.up |> clamp -1.0 1.0) |> abs
        dl <= spatialTol && af <= angleTol && ar <= angleTol && au <= angleTol


type CameraPose =
    struct
        val mutable public RotationIndex : int
        val mutable public ScaleSign : int
        val mutable public Rotation : M33d
        val mutable public Translation : V3d
        new(ri, ss, r, t) = { RotationIndex = ri; ScaleSign = ss; Rotation = r; Translation = t }
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
        CameraPose(pose.RotationIndex, s * pose.ScaleSign, pose.Rotation, f * pose.Translation)

    let tryFindScaled (srcCam : Camera) (worldObservations : list<V3d * V2d>) (pose : CameraPose) =
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


        let scales =
            let t = dst0Translation
            dst0Observations |> List.collect (fun (point, obs) ->
                // project(point + s * t) = obs
                
                // (point.xy + s * t.xy) / (point.z + s * t.z) = obs
                // point.xy + s * t.xy  = obs * (point.z + s * t.z)
                // point.xy + s * t.xy  = obs * point.z + s * obs * t.z
                // s * t.xy - s * obs * t.z  = obs * point.z - point.xy
                // s * (t.xy - obs * t.z) = obs * point.z - point.xy
                // s = (obs * point.z - point.xy) / (t.xy - obs * t.z)

                let s = (obs * point.Z - point.XY) / (t.XY - obs * t.Z)
                    
                [s.X; s.Y]
            )

        let scaleRange = Range1d scales
        if scaleRange.Size > 0.1 then
            //Log.warn "bad scale: %A +/- %A" scaleRange.Center (scaleRange.Size / 2.0)
            None
        else
            let s = List.average scales
            Some (scale -s pose)

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
        CameraPose(pose.RotationIndex, pose.ScaleSign, R', t')


[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module Ray =
    let intersection (r : seq<Ray3d>) = r.GetMiddlePoint()


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
        Native.cvRecoverPoses_(&&cfg, a.Length, a, b, &m1, &m2, &t, ms)
        (m1, m2, t, ms)

    let recoverPoses2 (cfg : RecoverPoseConfig) (a : V2d[]) (b : V2d[]) =
        let mutable m1 = M33d.Identity
        let mutable m2 = M33d.Identity
        let mutable t = V3d(100,123,432)
        let mutable cfg = cfg
        let mutable ms = Array.create a.Length 0uy
        Native.cvRecoverPoses_(&&cfg, a.Length, a, b, &m1, &m2, &t, ms)

        let m1 = m1.Transposed
        let m2 = m2.Transposed

        let possiblePoses =
            [
                CameraPose(0, 1, m1, t)
                CameraPose(1, 1, m2, t)
            ]

        let mask =
            Array.map ((<>) 0uy) ms

        possiblePoses, mask

open System.Threading

[<Struct; CustomEquality; CustomComparison>]
type CameraId private(id : int) =
    static let mutable current = 0

    member x.Id = id

    override x.ToString() =
        sprintf "c%d" id

    override x.GetHashCode() = id

    override x.Equals o =
        match o with
            | :? CameraId as o -> id = o.Id
            | _ -> false

    member x.CompareTo (o : CameraId) =
        compare id o.Id
            
    interface IComparable with
        member x.CompareTo o =
            match o with
                | :? CameraId as o -> compare id o.Id
                | _ -> failwith "uncomparabe"
            

    static member New = CameraId(Interlocked.Increment(&current))

[<Struct; CustomEquality; CustomComparison>]
type TrackId private(id : int) =
    static let mutable current = 0

    member x.Id = id

    override x.ToString() =
        sprintf "t%d" id

    override x.GetHashCode() = id

    override x.Equals o =
        match o with
            | :? TrackId as o -> id = o.Id
            | _ -> false

    member x.CompareTo (o : CameraId) =
        compare id o.Id
            
    interface IComparable with
        member x.CompareTo o =
            match o with
                | :? TrackId as o -> compare id o.Id
                | _ -> failwith "uncomparabe"
            

    static member New = TrackId(Interlocked.Increment(&current))

    module private MapExt =
        let intersect (l : MapExt<'k, 'a>) (r : MapExt<'k, 'b>) =
            MapExt.choose2 (fun _ l r -> match l, r with | Some l, Some r -> Some (l,r) | _ -> None) l r

        let values (r : MapExt<'k, 'v>) =
            r |> MapExt.toSeq |> Seq.map snd

type PhotoNetworkEdge =
    {
        left                : CameraId
        right               : CameraId
        leftObservations    : MapExt<TrackId, V2d>
        rightObservations   : MapExt<TrackId, V2d>

        leftToRight         : list<CameraPose>
        rightToLeft         : list<CameraPose>

        trackCount          : int
        tracks              : Set<TrackId>
    }

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module PhotoNetworkEdge =

    let create (lid : CameraId) (lObs : MapExt<TrackId, V2d>) (rid : CameraId) (rObs : MapExt<TrackId, V2d>) =
            
        let bothObs = MapExt.intersect lObs rObs
        let trackIds, observations = bothObs |> MapExt.toArray |> Array.unzip
        let l, r = Array.unzip observations

        let poses, mask = MiniCV.recoverPoses2 RecoverPoseConfig.Default l r

        let mutable trackCount = 0
        let mutable tracks = Set.empty
        for i in 0 .. mask.Length - 1 do
            if mask.[i] then
                tracks <- Set.add trackIds.[i] tracks
                trackCount <- trackCount + 1

        {
            left = lid
            right = rid
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
            leftObservations    = edge.rightObservations
            rightObservations   = edge.leftObservations

            leftToRight         = edge.rightToLeft
            rightToLeft         = edge.leftToRight

            trackCount          = edge.trackCount
            tracks              = edge.tracks
        }

type PhotoNetwork =
    {
        count               : int
        cameras             : MapExt<CameraId, Camera>
        edges               : MapExt<CameraId * CameraId, PhotoNetworkEdge>
        points              : MapExt<TrackId, V3d>
        observations        : MapExt<CameraId, MapExt<TrackId, V2d>>
    }

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module PhotoNetwork =
    let private rootCam = Camera.lookAt (V3d(0,-5,0)) V3d.Zero V3d.OOI
    let private firstDistance = 5.0
    let private minTrackCount = 5

    let private triangulatePoints (lid : CameraId) (lCam : Camera) (rid : CameraId) (rCam : Camera) (network : PhotoNetwork) =
        match MapExt.tryFind (lid, rid) network.edges with
            | Some edge ->
                edge.tracks 
                |> Seq.map (fun tid ->
                    let l = MapExt.find tid edge.leftObservations |> Camera.unproject1 lCam
                    let r = MapExt.find tid edge.rightObservations |> Camera.unproject1 rCam
                    tid, Ray.intersection [l;r]
                )
                |> MapExt.ofSeq

            | None ->
                MapExt.empty

    let private withPoints (net : PhotoNetwork) =
        let mutable measurements = MapExt.empty
        for (cid, cam) in MapExt.toSeq net.cameras do
            let obs = MapExt.tryFind cid net.observations |> Option.defaultValue MapExt.empty

            for (tid, pt) in MapExt.toSeq obs do
                measurements <- 
                    measurements |> MapExt.alter tid (fun ms ->
                        let ms = ms |> Option.defaultValue []
                        Some ((Camera.unproject1 cam pt) :: ms)
                    )

        let points = 
            measurements |> MapExt.choose (fun tid rays ->
                match rays with
                    | [] | [_] -> None
                    | _ -> 
                        let pt = Ray.intersection rays
                        Some pt
            )

        { net with points = points }


    let empty = 
        {
            count = 0
            cameras = MapExt.empty
            points = MapExt.empty
            edges = MapExt.empty
            observations = MapExt.empty
        }

    let getObservations (cid : CameraId) (network : PhotoNetwork) =
        MapExt.tryFind cid network.observations |> Option.defaultValue MapExt.empty

    let tryAdd (cid : CameraId) (observations : MapExt<TrackId, V2d>) (network : PhotoNetwork) =

        if network.count = 0 then
            Some { 
                network with
                    count = network.count + 1
                    cameras = MapExt.add cid rootCam network.cameras
                    observations = MapExt.add cid observations network.observations
            }

        elif network.count = 1 then
            let pid, parent = network.cameras |> MapExt.toSeq |> Seq.head
            let parentObs = getObservations pid network

            let edge = PhotoNetworkEdge.create pid parentObs cid observations

            if edge.trackCount >= minTrackCount then
                let camera = 
                    let t = edge.leftToRight |> List.head |> CameraPose.scale firstDistance |> CameraPose.transformation
                    Camera.transformedView t parent

                let newNetwork =
                    { network with
                        count = 2
                        cameras = MapExt.add cid camera network.cameras
                        edges = MapExt.add (pid, cid) edge (MapExt.add (cid, pid) (PhotoNetworkEdge.inverse edge) network.edges)
                        observations = MapExt.add cid observations network.observations
                    }

                Some (withPoints newNetwork)
            else
                None

        elif network.count = 2 then
            let candidates = 
                network.observations |> MapExt.toSeq |> Seq.map (fun (pid, parentObs) ->
                    let edge = PhotoNetworkEdge.create pid parentObs cid observations
                    (pid, cid), edge
                )
                |> MapExt.ofSeq

            let ((c1, c2), e12) = candidates |> MapExt.toSeq |> Seq.maxBy (fun (_,e) -> e.trackCount)

            if e12.trackCount >= minTrackCount then
                let c0 = network.cameras |> MapExt.remove c1 |> MapExt.toSeq |> Seq.head |> fst
                let e01 = MapExt.find (c0, c1) network.edges

                let cam0 = rootCam

                let configurations = 
                    e01.leftToRight |> List.collect (fun p01 ->
                        let cam1 = 
                            let t = p01 |> CameraPose.scale firstDistance |> CameraPose.transformation
                            Camera.transformedView t cam0

                        let points =
                            let worldPoints = triangulatePoints c0 cam0 c1 cam1 network
                            MapExt.intersect worldPoints observations
                                |> MapExt.toList
                                |> List.map snd


                        e12.leftToRight |> List.collect (fun p12 ->
                            match CameraPose.tryFindScaled cam1 points p12 with
                                | Some p12 ->
                                    let cam2 = Camera.transformedView (CameraPose.transformation p12) cam1
                                    [(cam0, cam1, cam2)]

                                | None ->
                                    []
                        )
                    )


                match configurations with
                    | (cam0, cam1, cam2) :: _ ->
                        
                        let forwardEdges = candidates
                        let backwardEdges = candidates |> MapExt.toSeq |> Seq.map (fun ((l,r),e) -> (r,l), PhotoNetworkEdge.inverse e) |> MapExt.ofSeq

                        let newNetwork =
                            { network with
                                count = 3
                                cameras = MapExt.ofList [c0, cam0; c1, cam1; c2, cam2]
                                edges = MapExt.union network.edges (MapExt.union forwardEdges backwardEdges)
                                observations = MapExt.add cid observations network.observations
                            }

                        Some (withPoints newNetwork)

                    | [] ->
                        None
            else
                None

        else
            let candidates = 
                network.observations |> MapExt.toSeq |> Seq.map (fun (pid, parentObs) ->
                    let edge = PhotoNetworkEdge.create pid parentObs cid observations
                    (pid, cid), edge
                )
                |> MapExt.ofSeq

            let ((c1, c2), e12) = candidates |> MapExt.toSeq |> Seq.maxBy (fun (_,e) -> e.trackCount)
            
            if e12.trackCount >= minTrackCount then
                let points = MapExt.intersect network.points observations |> MapExt.toList |> List.map snd
                let cam1 = MapExt.find c1 network.cameras
                let configurations =
                    e12.leftToRight |> List.choose (fun p12 ->
                        match CameraPose.tryFindScaled cam1 points p12 with
                            | Some e12 ->
                                Camera.transformedView (CameraPose.transformation e12) cam1 |> Some
                            | _ ->
                                None
                    )

                match configurations with
                    | cam2 :: _ ->
                        
                        let forwardEdges = candidates
                        let backwardEdges = candidates |> MapExt.toSeq |> Seq.map (fun ((l,r),e) -> (r,l), PhotoNetworkEdge.inverse e) |> MapExt.ofSeq

                        let newNetwork =
                            { network with
                                count = network.count + 1
                                cameras = MapExt.add c2 cam2 network.cameras
                                edges = MapExt.union network.edges (MapExt.union forwardEdges backwardEdges)
                                observations = MapExt.add c2 observations network.observations
                            }

                        Some (withPoints newNetwork)

                    | [] ->
                        None 


            else
                None

    let add (cid : CameraId) (observations : MapExt<TrackId, V2d>) (network : PhotoNetwork) =
        match tryAdd cid observations network with
            | Some n -> n
            | _ -> network

module Test =
    let rand = RandomSystem()


    
    let testRays() =
        let points =
            Array.init 10000 (fun _ ->
                rand.UniformV3dDirection()
            )


        let c0 = Camera.lookAt (rand.UniformV3dDirection() * 5.0) V3d.Zero V3d.OOI
        let c1 = Camera.lookAt (rand.UniformV3dDirection() * 5.0) V3d.Zero V3d.OOI

        let p0 = points |> Array.map (Camera.project1 c0)
        let p1 = points |> Array.map (Camera.project1 c1)

        let r0 = p0 |> Array.map (Camera.unproject1 c0)
        let r1 = p1 |> Array.map (Camera.unproject1 c1)

        let results = 
            Array.zip r0 r1
                |> Array.map (fun (l,r) -> Ray.intersection [l;r])

        let errorCount =
            Array.zip points results
                |> Array.filter (fun (pReal, pObs) -> not (V3d.ApproxEqual(pReal, pObs, 1.0E-10)))
                |> Array.length

        if errorCount = 0 then
            Log.line "Success"
        else
            Log.warn "%d Errors" errorCount

        
    let run() =
        // init some random points
        let points =
            Array.init 10000 (fun _ ->
                rand.UniformV3dDirection() * (rand.UniformDouble() * 2.0)
            )


        for i in 1 .. 1000 do
            // create two random cameras
            let c0 = Camera.lookAt (rand.UniformV3dDirection() * (2.0 + rand.UniformDouble() * 6.0)) V3d.Zero V3d.OOI
            let c1 = Camera.lookAt (rand.UniformV3dDirection() * (2.0 + rand.UniformDouble() * 6.0)) V3d.Zero V3d.OOI
            let c2 = Camera.lookAt (rand.UniformV3dDirection() * (2.0 + rand.UniformDouble() * 6.0)) V3d.Zero V3d.OOI
            let c3 = Camera.lookAt (rand.UniformV3dDirection() * (2.0 + rand.UniformDouble() * 6.0)) V3d.Zero V3d.OOI

            let tracks = points |> Array.map (fun _ -> TrackId.New)

            let observe (c : Camera) =
                points |> Seq.mapi (fun i p ->
                    let tid = tracks.[i]
                    let c = Camera.project1 c p
                    tid, c
                ) |> MapExt.ofSeq

            let net = 
                PhotoNetwork.empty
                    |> PhotoNetwork.add CameraId.New (observe c0)
                    |> PhotoNetwork.add CameraId.New (observe c1)
                    |> PhotoNetwork.add CameraId.New (observe c2)
                    |> PhotoNetwork.add CameraId.New (observe c3)


            // project a set of points
            let p0 = points |> Array.map (Camera.project1 c0)
            let p1 = points |> Array.map (Camera.project1 c1)

            // recover the possible poses from 2D observations
            let (poses, mask) = MiniCV.recoverPoses2 RecoverPoseConfig.Default p0 p1

            // use the real points as reference-system for the scale
            let observations = 
                mask 
                |> Seq.mapi (fun i m ->
                    if m then Some (points.[i], p1.[i])
                    else None
                )
                |> Seq.choose id
                |> Seq.toList


            // find all poses that agree with the 3D points
            let properPoses =
                poses |> List.choose (fun pose ->
                    // try to determine a scale for the pose-translation (based on the 3D points)
                    match CameraPose.tryFindScaled c0 observations pose with
                        | Some pose ->

                            let name = pose |> CameraPose.name
                            let trafo = pose |> CameraPose.transformation
                            
                            // create the transformed camera (which should be equal to c1)
                            let test = c0 |> Camera.transformedView trafo

                            // if the test-camera is equal to c1 return the pose 
                            if Camera.approxEqual 0.01 0.01 c1 test then
                                Some pose
                            else
                                None
                        | None ->
                            None
                )

            match properPoses with
                | [] ->
                    
                    Log.start "Error"
                    let observations = Array.zip points p1 |> Array.toList
                    for p in poses do
                        match CameraPose.tryFindScaled c0 observations p with
                            | Some p -> 
                                let name = CameraPose.name p
                                let trafo = CameraPose.transformation p
                                let test = c0 |> Camera.transformedView trafo
                                let a = Camera.angles c1 test
                                let d = Camera.distance c1 test
                                Log.warn "%s: { angles: %A; distance: %A }" name a d
                                ()
                            | None ->
                                let name = CameraPose.name p
                                Log.warn "%s: bad scale" name

                    Log.stop()
                | t ->
                    Log.line "Success: %A" (List.map CameraPose.name t)


        ()

open MiniCV
module bagd =
    open Aardvark.Base
    open Aardvark.Base.Rendering

        
    
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



    [<EntryPoint>]
    let main argv = 
        Ag.initialize()
        Aardvark.Init()
        Test.run()
        Environment.Exit 0


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

            let ( r1a, r2a, t, _ ) = recoverPoses cfg obs0 obs1
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

 





