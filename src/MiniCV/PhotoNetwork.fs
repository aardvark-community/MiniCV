namespace Aardvark.Reconstruction

open System
open Aardvark.Base
open Aardvark.Reconstruction



[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module PhotoNetworkConfig =
    let Default =
        {
            inlierThreshold     = 0.005
            rootCam             = Camera.lookAt (V3d(0.0, 5.0, 0.0)) V3d.OOO V3d.OOI V2d.II
            firstDistance       = 5.0
            ndcTolerance        = Double.PositiveInfinity // => 0.05 => 5% => 50px@1k
        }

    let withFocalLength (f : V2d) (cfg : PhotoNetworkConfig) =
        {
            cfg with
                rootCam = { cfg.rootCam with focal = f }
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
                    match pt with
                        | None -> None
                        | Some pt -> Some (tid, pt)
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
                        match pt with
                            | None -> None
                            | Some pt -> Some (tid, pt)
            ) |> MapExt.ofSeq

        let cost = 
            points |> MapExt.toSeq 
                   |> Seq.sumBy ( fun (tid, point) ->
                        let ms = !measurements.GetOrCreate(tid, fun _ -> ref [])
                        ms |> List.sumBy (fun (_,cam,obs) ->
                                match point |> Camera.project1 cam with
                                    | Some real -> 
                                        Vec.lengthSquared (obs - real)
                                    | None ->
                                        0.0
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

            let edge = 
                match MapExt.tryFind (pid, cid) network.edges with
                    | Some e -> e
                    | _ -> PhotoNetworkEdge.create network.config pid parentObs cid observations

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

                        let ownEdges = MapExt.ofList [(pid, cid), edge; (cid, pid), PhotoNetworkEdge.inverse edge]

                        let newNetwork =
                            { network with
                                count = 2
                                cameras = MapExt.add cid camera network.cameras
                                edges = MapExt.union network.edges ownEdges
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
                    let edge = 
                        match MapExt.tryFind (pid, cid) network.edges with
                            | Some e -> e
                            | _ -> PhotoNetworkEdge.create network.config pid parentObs cid observations
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

                   
                        let worldPoints = triangulatePoints c0 cam0 c1 cam1 network
                        let points =
                            MapExt.intersect worldPoints observations
                                |> MapExt.toList
                                |> List.map snd

                        let passt =
                            worldPoints |> MapExt.values |> Seq.forall ( fun p -> Vec.dot (p - (cam1.location)).Normalized cam1.forward >= 0.0 )
                        

                        if passt then
                            e12.leftToRight |> List.collect (fun p12 ->
                                let (cost, p12) = CameraPose.findScaled network.config.inlierThreshold cam1 points p12
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
                    let edge = 
                        match MapExt.tryFind (pid, cid) network.edges with
                            | Some e -> e
                            | _ -> PhotoNetworkEdge.create network.config pid parentObs cid observations
                    (pid, cid), edge
                )
                |> MapExt.ofSeq

            let ((c1, c2), e12) = candidates |> MapExt.toSeq |> Seq.maxBy (fun (_,e) -> e.trackCount)
            
            if e12.trackCount >= minTrackCount then
                let points = MapExt.intersect network.points observations |> MapExt.toList |> List.map snd
                let cam1 = MapExt.find c1 network.cameras
                let configurations =
                    e12.leftToRight |> List.map (fun p12 ->
                        let (cost, e12) = CameraPose.findScaled network.config.inlierThreshold cam1 points p12
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
      

    type private UnionNode(id : CameraId) as this =
        let mutable parent = this
        let mutable rank = 1

        member x.Parent
            with get() = parent
            and set p = parent <- p

        member x.Rank
            with get() = rank
            and set r = rank <- r

        member x.Id = id

    type private UnionFind() =
        let nodes = Dict()

        let node (id : CameraId) = nodes.GetOrCreate(id, fun id -> UnionNode(id))

        let rec rep (n : UnionNode) =
            if n.Parent = n then
                n
            else
                let r = rep n.Parent
                n.Parent <- r
                r

        member x.Add(li : CameraId, ri : CameraId) =
            let ln = node li
            let rn = node ri

            let lr = rep ln
            let rr = rep rn

            if lr = rr then
                false
            else
                if lr.Rank < rr.Rank then
                    lr.Parent <- rr
                    true
                elif rr.Rank < lr.Rank then
                    rr.Parent <- lr.Parent
                    true
                else
                    rr.Parent <- lr.Parent
                    lr.Rank <- lr.Rank + 1
                    true

    type Tree<'a> =
        | Empty
        | Node of 'a * list<Tree<'a>>

    let private edgeForest (edges : list<(CameraId * CameraId) * PhotoNetworkEdge>) =
        let uf = UnionFind()
        let edges = edges |> List.sortByDescending (fun (_,e) -> e.trackCount)

        let adjacency = Dict<CameraId, Dict<CameraId, PhotoNetworkEdge>>()
        let nodes = System.Collections.Generic.HashSet<CameraId>()

        let addEdge (l : CameraId) (r : CameraId) (e : PhotoNetworkEdge) =
            let ln = adjacency.GetOrCreate(l, fun _ -> Dict())
            let rn = adjacency.GetOrCreate(r, fun _ -> Dict())
            ln.[r] <- e
            rn.[l] <- PhotoNetworkEdge.inverse e


        for ((l,r),e) in edges do
            nodes.Add l |> ignore
            nodes.Add r |> ignore
            if uf.Add(l,r) then
                addEdge l r e


        

        [
            while nodes.Count > 0 do
                let fst = nodes |> Seq.head

                let rec traverse (n : CameraId) =
                    if nodes.Remove n then
                        let children = 
                            match adjacency.TryGetValue(n) with
                                | (true, ns) ->
                                    ns.Keys |> Seq.toList |> List.collect traverse
                                | _ ->
                                    []

                        [Node(n, children)]
                    else
                        []
                   
                let tree = traverse fst
                yield! tree
        ]


    let ofMap (cfg : PhotoNetworkConfig) ( cams : MapExt<CameraId, MapExt<TrackId, V2d>> ) =
        Log.startTimed "creating network"

        Log.startTimed "creating edges"
        Report.Progress(0.0)
        let edges =
            let camsArr = cams |> MapExt.toArray
            let cnt = (camsArr.Length * (camsArr.Length - 1)) / 2
            let step = 1.0 / float cnt
            [
                for i in 0 .. camsArr.Length - 1 do
                    for j in i + 1 .. camsArr.Length - 1 do
                        let (ci, oi) = camsArr.[i]
                        let (cj, oj) = camsArr.[j]

                        Report.ProgressDelta(step)
                        let e = PhotoNetworkEdge.create cfg ci oi cj oj
                        yield (ci, cj), e
            ]

        Log.stop()

        let edgeCache =
            edges |> List.collect (fun ((l,r),e) -> [ (l,r),e; ((r,l), PhotoNetworkEdge.inverse e) ] ) |> MapExt.ofList

        let empty = { empty cfg with edges = edgeCache }

        Log.startTimed "sorting cameras"
        let rec flatten (t : Tree<CameraId>) =
            match t with
                | Empty -> []
                | Node(a,c) -> (a, cams.[a]) :: (c |> List.collect flatten)

        let order = edgeForest edges |> List.collect flatten

        Log.stop()


        let rec epoch network cs ps nps =
            match cs with
            | [] -> 
                if ps > 0 then
                    epoch network (List.rev nps) 0 []
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
                let (network, good, remaining) = epoch empty cams 0 []
                addMany (network::networks) (List.rev remaining)

        let res = addMany [] order |> List.sortByDescending (fun n -> n.count)
        Log.stop()
        res
        
    let ofList (cfg : PhotoNetworkConfig) (cams : list<CameraId * MapExt<TrackId, V2d>>) =
        cams |> MapExt.ofList |> ofMap cfg

    let tracks (net : PhotoNetwork) =
        let mutable measurements = MapExt.empty

        for camPair in net.usedEdges do
            let (l,r) = camPair
            let edge = net.edges.[camPair]

            let obs = MapExt.intersect edge.leftObservations edge.rightObservations
            
            for (tid, (lObs,rObs)) in MapExt.toSeq obs do
                let ms =
                    match MapExt.tryFind tid measurements with
                        | Some r -> r
                        | None ->
                            let r = ref MapExt.empty
                            measurements <- MapExt.add tid r measurements
                            r
                ms :=
                    !ms 
                        |> MapExt.add l lObs
                        |> MapExt.add r rObs

        measurements |> MapExt.map (fun _ r -> !r)

    let transformed (t : Trafo3d) (net : PhotoNetwork) =
        { net with
            cameras = net.cameras |> MapExt.map (fun _ -> Camera.transformed t)
            points = net.points |> MapExt.map (fun _ -> t.Forward.TransformPos)
        }





