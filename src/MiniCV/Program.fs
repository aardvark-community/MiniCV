﻿namespace Aardvark.Reconstruction

open System
open Aardvark.Base

open System.Collections.Generic
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
            RecoverPoseConfig(1.0, V2d.Zero, 0.999, 0.01)

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
    let private projTrafo (n : float) (f : float) : Trafo3d = 
        Trafo3d(
            M44d(
                                1.0,                     0.0,                       0.0,                        0.0,
                                0.0,                     1.0,                       0.0,                        0.0,
                                0.0,                     0.0,         (f + n) / (n - f),    (2.0 * f * n) / (n - f),
                                0.0,                     0.0,                      -1.0,                        0.0
                ),                                                     
                                                                       
            M44d(                                      
                                1.0,                     0.0,                       0.0,                       0.0,
                                0.0,                     1.0,                       0.0,                       0.0,
                                0.0,                     0.0,                       0.0,                      -1.0,
                                0.0,                     0.0,   (n - f) / (2.0 * f * n),     (f + n) / (2.0 * f * n)
                )
        )
    
    let viewTrafo (c : Camera) =
        let z = -c.forward
        let x = c.right
        let y = c.up
        let p = c.location
        Trafo3d.FromBasis(x,y,z,p).Inverse

    let viewProjTrafo1 (n : float) (f : float) (c : Camera) =
        let view = viewTrafo c
        let proj = projTrafo n f
        view * proj

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

    let findScaled (srcCam : Camera) (worldObservations : list<V3d * V2d>) (pose : CameraPose) =
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

            let scales =
                let t = dst0Translation
                worldObservations |> List.collect (fun (worldPoint, obs) ->
                    let point = dst0View.Forward.TransformPos(worldPoint)
                    // project(point + s * t) = obs
                
                    // (point.xy + s * t.xy) / (point.z + s * t.z) = obs
                    // point.xy + s * t.xy  = obs * (point.z + s * t.z)
                    // point.xy + s * t.xy  = obs * point.z + s * obs * t.z
                    // s * t.xy - s * obs * t.z  = obs * point.z - point.xy
                    // s * (t.xy - obs * t.z) = obs * point.z - point.xy
                    // s = (obs * point.z - point.xy) / (t.xy - obs * t.z)
                    let z = (obs * point.Z - point.XY) 
                    let n = t.XY - obs * t.Z

                    let nt = Fun.IsTiny(n.X, 1E-5) || Fun.IsTiny(n.Y, 1E-5)

                    match nt with
                        | true -> []
                        | _ -> 
                            let s = -z / n

                            [s.X; s.Y]


                )

            let scaleRange = Range1d scales
            let s = List.average scales

            let finalPose = scale s pose
            let dstCam = Camera.transformedView (transformation finalPose) srcCam

            let mutable cnt = 0
            let cost = worldObservations |> List.sumBy (fun (w,o) -> cnt <- cnt + 1; Camera.project1 dstCam w - o |> Vec.lengthSquared)
            let avgCost = sqrt (cost / float cnt)

            avgCost, finalPose

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


[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module Ray =
    let rec intersections (r : list<Ray3d>) = 
        match r with
            | a :: rest ->
                let toRest =
                    rest |> List.choose (fun b ->
                        if V3d.ApproxEqual(a.Direction, b.Direction) then
                            None
                        else
                            let pt = a.GetMiddlePoint(b)
                            if pt.AnyNaN || pt.AnyInfinity then
                                None
                            else
                                let d = a.GetMinimalDistanceTo(pt) + b.GetMinimalDistanceTo(pt)
                                Some (d,pt)
                    )


                toRest @ intersections rest
            | _ ->
                []

    let intersection (r : list<Ray3d>) =
        let r = HSet.ofList r |> HSet.toList
        r.GetMiddlePoint()
//        let candidates = intersections r |> List.sortBy fst
//        match candidates with
//            | [] -> failwith "not possible"
//            | [(_,p)] -> p
//            | ps -> 
//                let (n, p) = ps |> List.fold (fun (n,v) (_,p) -> (n + 1, v + p)) (0, V3d.Zero)
//                p / float n


module MiniCV =

    module Native =
        [<Literal>]
        let lib = @"MiniCVNative"
        
        [<DllImport(lib, EntryPoint = "cvRecoverPose"); SuppressUnmanagedCodeSecurity>]
        extern int cvRecoverPose_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat, V3d& tVec, byte[] ms)
        
        [<DllImport(lib, EntryPoint = "cvRecoverPoses"); SuppressUnmanagedCodeSecurity>]
        extern bool cvRecoverPoses_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat1, M33d& rMat2, V3d& tVec, byte[] ms)

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

[<AutoOpen>]
module ``Move to base once again`` =
    let inline tup a b = (a,b)

module private Option =
    let map2 (f : 'a -> 'b -> 'c) (a : Option<'a>) (b : Option<'b>) =
        match a with
            | Some a ->
                match b with
                    | Some b -> Some (f a b)
                    | None -> None
            | None -> None

module MapExt =
    let intersect (l : MapExt<'k, 'a>) (r : MapExt<'k, 'b>) =
        MapExt.choose2 (constF (Option.map2 tup)) l r

    let values (r : MapExt<'k, 'v>) =
        r |> MapExt.toSeq |> Seq.map snd

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

            let poses, mask = MiniCV.recoverPoses2 RecoverPoseConfig.Default l r

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

type PhotoNetworkConfig =
    {
        recoverPoseConfig   : RecoverPoseConfig
        rootCam             : Camera
        firstDistance       : float
        ndcTolerance        : float
    }

[<CustomEquality; NoComparison>]
type PhotoNetwork =
    {
        config              : PhotoNetworkConfig
        count               : int
        cost                : float
        cameras             : MapExt<CameraId, Camera>
        edges               : MapExt<CameraId * CameraId, PhotoNetworkEdge>
        usedEdges           : Set<CameraId * CameraId>
        points              : MapExt<TrackId, V3d>
        observations        : MapExt<CameraId, MapExt<TrackId, V2d>>
    }

    override x.GetHashCode() = HashCode.Combine(x.config.GetHashCode(), x.cameras.GetHashCode())
    override x.Equals o =
        match o with
            | :? PhotoNetwork as o -> x.config = o.config && x.cameras = o.cameras
            | _ -> false

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module PhotoNetworkConfig =
    let Default =
        {
            recoverPoseConfig   = RecoverPoseConfig.Default
            rootCam             = Camera.lookAt (V3d(0.0, 5.0, 0.0)) V3d.OOO V3d.OOI
            firstDistance       = 5.0
            ndcTolerance        = Double.PositiveInfinity // => 0.05 => 5% => 50px@1k
        }

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module PhotoNetwork =
    let private minTrackCount = 5

    let private triangulatePoints (lid : CameraId) (lCam : Camera) (rid : CameraId) (rCam : Camera) (network : PhotoNetwork) =
        match MapExt.tryFind (lid, rid) network.edges with
            | Some edge ->
                edge.tracks 
                |> Seq.choose (fun tid ->
                    let l = MapExt.find tid edge.leftObservations |> Camera.unproject1 lCam
                    let r = MapExt.find tid edge.rightObservations |> Camera.unproject1 rCam
                    let pt =  Ray.intersection [l;r]
                    if pt.AnyInfinity || pt.AnyNaN then
                        None
                    else
                        Some (tid, pt)
                )
                |> MapExt.ofSeq

            | None ->
                MapExt.empty

    let private postProcess (net : PhotoNetwork) =
        let measurements = Dict()

        for camPair in net.usedEdges do
            let (l,r) = camPair
            let edge = net.edges.[camPair]

            let obs = MapExt.intersect edge.leftObservations edge.rightObservations
            let lCam = net.cameras.[l]
            let rCam = net.cameras.[r]

            for (tid, (lObs,rObs)) in MapExt.toSeq obs do
                let ms = measurements.GetOrCreate(tid, fun _ -> ref [])

                ms :=
                    (Camera.unproject1 lCam lObs, lCam, lObs) ::
                    (Camera.unproject1 rCam rObs, rCam, rObs) ::
                    !ms

        let points = 
            measurements |> Dict.toSeq |> Seq.choose (fun (tid, rays) ->
                match !rays with
                    | [] | [_] -> None
                    | rays -> 
                        
                        let rays = rays |> List.map ( fun (r,_,_) -> r )
                        let pt = Ray.intersection rays
                        if pt.AnyInfinity || pt.AnyNaN then
                            None
                        else
                            Some (tid, pt)
            ) |> MapExt.ofSeq

        let cost = 
            points |> MapExt.toSeq 
                   |> Seq.sumBy ( fun (tid, point) ->
                        let ms = !measurements.GetOrCreate(tid, fun _ -> ref [])
                        ms |> List.sumBy (fun (_,cam,obs) ->
                                let real = point |> Camera.project1 cam
                                Vec.lengthSquared (obs - real)
                           )
                   )

        { net with points = points; cost = 0.5 * cost }


    let empty cfg = 
        {
            config = cfg
            count = 0
            cost = 0.0
            cameras = MapExt.empty
            points = MapExt.empty
            edges = MapExt.empty
            usedEdges = Set.empty
            observations = MapExt.empty
        }

    let getObservations (cid : CameraId) (network : PhotoNetwork) =
        MapExt.tryFind cid network.observations |> Option.defaultValue MapExt.empty

    let remove (cid : CameraId) (network : PhotoNetwork) =
        match MapExt.tryFind cid network.cameras with
            | Some _ ->
                postProcess 
                    { network with
                        count = network.count - 1
                        cameras = MapExt.remove cid network.cameras
                        edges = network.edges |> MapExt.filter (fun (l,r) _ -> l <> cid && r <> cid)
                        observations = MapExt.remove cid network.observations
                    }
            | None ->
                network

    let tryAdd (cid : CameraId) (observations : MapExt<TrackId, V2d>) (network : PhotoNetwork) =
        let network = remove cid network
        if network.count = 0 then
            Some { 
                network with
                    count = 1
                    cameras = MapExt.add cid network.config.rootCam network.cameras
                    observations = MapExt.add cid observations network.observations
            }

        elif network.count = 1 then
            let pid, parent = network.cameras |> MapExt.toSeq |> Seq.head
            let parentObs = getObservations pid network

            let edge = PhotoNetworkEdge.create pid parentObs cid observations

            if edge.trackCount >= minTrackCount then
                
                let poses =
                    edge.leftToRight |> List.collect (fun p ->
                        [
                            p |> CameraPose.scale network.config.firstDistance  
                            p |> CameraPose.scale -network.config.firstDistance    
                        ]
                    )
                    

                let configurations = 
                    poses |> List.choose (fun p ->
                        let cam1 = 
                            let t = p |> CameraPose.transformation
                            Camera.transformedView t parent

                        let points =
                            let worldPoints = triangulatePoints pid parent cid cam1 network
                            MapExt.intersect worldPoints observations
                                |> MapExt.toList
                                |> List.map snd

                        let passt =
                            points |> List.forall ( fun (p,_) -> Vec.dot (p - (cam1.location)).Normalized cam1.forward >= 0.0 )

                        if passt then
                            Some p
                        else
                            None
                    )

                match configurations with
                    | pose :: _ ->
                        let camera = 
                            let t = pose |> CameraPose.transformation
                            Camera.transformedView t parent

                        let newNetwork =
                            { network with
                                count = 2
                                cameras = MapExt.add cid camera network.cameras
                                edges = MapExt.ofList [(pid, cid), edge; (cid, pid), PhotoNetworkEdge.inverse edge]
                                usedEdges = Set.ofList [(pid, cid)]
                                observations = MapExt.add cid observations network.observations
                            }

                        Some (postProcess newNetwork)
                    | _ ->
                        None
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

                let cam0 = network.config.rootCam

                let e01Poses =
                        e01.leftToRight |> List.collect (fun p ->
                            [
                                p |> CameraPose.scale network.config.firstDistance  
                                p |> CameraPose.scale -network.config.firstDistance    
                            ]
                        )

                let configurations = 
                    e01Poses |> List.collect (fun p01 ->
                        let cam1 = 
                            let t = p01 |> CameraPose.transformation
                            Camera.transformedView t cam0

                        let points =
                            let worldPoints = triangulatePoints c0 cam0 c1 cam1 network
                            MapExt.intersect worldPoints observations
                                |> MapExt.toList
                                |> List.map snd

                        let passt =
                            points |> List.forall ( fun (p,_) -> Vec.dot (p - (cam1.location)).Normalized cam1.forward >= 0.0 )
                        

                        if passt then
                            e12.leftToRight |> List.collect (fun p12 ->
                                let (cost, p12) = CameraPose.findScaled cam1 points p12
                                let cam2 = Camera.transformedView (CameraPose.transformation p12) cam1
                                    
                                [(cost, cam0, cam1, cam2)]
                            )
                        else
                            []
                    )


                match configurations with
                    | [] -> None
                    | l ->
                        let (cost, cam0, cam1, cam2) = List.minBy (fun (c,_,_,_) -> c) l
                        
                        if cost > network.config.ndcTolerance then
                            None
                        else
                            let forwardEdges = candidates
                            let backwardEdges = candidates |> MapExt.toSeq |> Seq.map (fun ((l,r),e) -> (r,l), PhotoNetworkEdge.inverse e) |> MapExt.ofSeq

                            let newNetwork =
                                { network with
                                    count = 3
                                    cameras = MapExt.ofList [c0, cam0; c1, cam1; c2, cam2]
                                    edges = MapExt.union network.edges (MapExt.union forwardEdges backwardEdges)
                                    usedEdges = Set.ofList [ (c0,c1); (c1,c2) ]
                                    observations = MapExt.add cid observations network.observations
                                }

                            Some (postProcess newNetwork)

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
                    e12.leftToRight |> List.map (fun p12 ->
                        let (cost, e12) = CameraPose.findScaled cam1 points p12
                        cost, Camera.transformedView (CameraPose.transformation e12) cam1
                    )

                match configurations with
                    | [] -> None
                    | l ->
                        let (cost, cam2) = List.minBy (fun (c,_) -> c) l
                        if cost > network.config.ndcTolerance then
                            None
                        else
                            let forwardEdges = candidates
                            let backwardEdges = candidates |> MapExt.toSeq |> Seq.map (fun ((l,r),e) -> (r,l), PhotoNetworkEdge.inverse e) |> MapExt.ofSeq

                            let newNetwork =
                                { network with
                                    count = network.count + 1
                                    cameras = MapExt.add c2 cam2 network.cameras
                                    edges = MapExt.union network.edges (MapExt.union forwardEdges backwardEdges)
                                    usedEdges = Set.add (c1, c2) network.usedEdges
                                    observations = MapExt.add c2 observations network.observations
                                }

                            Some (postProcess newNetwork)

            else
                None

    let add (cid : CameraId) (observations : MapExt<TrackId, V2d>) (network : PhotoNetwork) =
        match tryAdd cid observations network with
            | Some n -> n
            | _ -> network
      
    let ofList (cfg : PhotoNetworkConfig) ( cams : list<CameraId * MapExt<TrackId, V2d>> ) =
        
        let rec epoch network cs ps nps =
            match cs with
            | [] -> 
                if ps > 0 then
                    epoch network nps 0 []
                else
                    network, ps, nps

            | (cid,obs)::remaining ->
                match tryAdd cid obs network with
                | Some n -> 
                    epoch n remaining (ps+1) nps
                | None -> 
                    epoch network remaining ps ((cid,obs)::nps)

        let rec addMany networks cams =
            match cams with
            | [] -> networks
            | _ ->
                let (network, good, remaining) = epoch (empty cfg) cams 0 []
                addMany (network::networks) remaining

        addMany [] cams |> List.sortByDescending (fun n -> n.count)
        
    let transformed (t : Trafo3d) (net : PhotoNetwork) =
        { net with
            cameras = net.cameras |> MapExt.map (fun _ -> Camera.transformed t)
            points = net.points |> MapExt.map (fun _ -> t.Forward.TransformPos)
        }





