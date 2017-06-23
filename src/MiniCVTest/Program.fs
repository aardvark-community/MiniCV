
open Aardvark.Base
open Aardvark.Base.Rendering
open Aardvark.Base.Incremental
open Aardvark.SceneGraph
open Aardvark.Application.WinForms
open Aardvark.Application
open Aardvark.Rendering.Text


open Aardvark.Reconstruction

let rand = RandomSystem()
    
let testRays() =
    let points =
        Array.init 1000 (fun _ ->
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
//
//let testRegister() =
//    for i in 1 .. 1000 do
//
//        let points =
//            Array.init 1000 (fun _ ->
//                rand.UniformV3dDirection() * (rand.UniformDouble() * 2.0)
//            )
//
//        let c0 = Camera.lookAt (rand.UniformV3dDirection() * (2.0 + rand.UniformDouble() * 6.0)) V3d.Zero V3d.OOI
//        let c1 = Camera.lookAt (rand.UniformV3dDirection() * (2.0 + rand.UniformDouble() * 6.0)) V3d.Zero V3d.OOI
//
//        // project a set of points
//        let p0 = points |> Array.map (Camera.project1 c0)
//        let p1 = points |> Array.map (Camera.project1 c1)
//
//        // recover the possible poses from 2D observations
//        let (poses, mask) = MiniCV.recoverPoses2 RecoverPoseConfig.Default p0 p1
//
//        // use the real points as reference-system for the scale
//        let observations = 
//            mask 
//            |> Seq.mapi (fun i m ->
//                if m then Some (points.[i], p1.[i])
//                else None
//            )
//            |> Seq.choose id
//            |> Seq.toList
//
//
//        // find all poses that agree with the 3D points
//        let properPoses =
//            poses |> List.choose (fun pose ->
//                // try to determine a scale for the pose-translation (based on the 3D points)
//                match CameraPose.tryFindScaled c0 observations pose with
//                    | Some pose ->
//
//                        let name = pose |> CameraPose.name
//                        let trafo = pose |> CameraPose.transformation
//                            
//                        // create the transformed camera (which should be equal to c1)
//                        let test = c0 |> Camera.transformedView trafo
//
//                        // if the test-camera is equal to c1 return the pose 
//                        if Camera.approxEqual 0.01 0.01 c1 test then
//                            Some pose
//                        else
//                            None
//                    | None ->
//                        None
//            )
//
//        match properPoses with
//            | [] ->
//                    
//                Log.start "Error"
//                let observations = Array.zip points p1 |> Array.toList
//                for p in poses do
//                    match CameraPose.tryFindScaled c0 observations p with
//                        | Some p -> 
//                            let name = CameraPose.name p
//                            let trafo = CameraPose.transformation p
//                            let test = c0 |> Camera.transformedView trafo
//                            let a = Camera.angles c1 test
//                            let d = Camera.distance c1 test
//                            Log.warn "%s: { angles: %A; distance: %A }" name a d
//                            ()
//                        | None ->
//                            let name = CameraPose.name p
//                            let trafo = CameraPose.transformation p
//                            let test = c0 |> Camera.transformedView trafo
//                            let a = Camera.angles c1 test
//
//                            Log.warn "%s: bad scale: { angles: %A }" name a
//
//                Log.stop()
//            | t ->
//                Log.line "Success: %A" (List.map CameraPose.name t)     
//                
let testNetworks() =
    let points =
        Array.init 1000 (fun _ ->
            rand.UniformV3dDirection() * (rand.UniformDouble() * 2.0)
        )


    for i in 1 .. 1000 do
        let cameras = 
            List.init 10 ( fun _ ->
                CameraId.New, Camera.lookAt (rand.UniformV3dDirection() * (2.0 + rand.UniformDouble() * 6.0)) (rand.UniformV3dDirection()) (rand.UniformV3dDirection())
            )
            
        let cameras2 = 
            List.init 10 ( fun _ ->
                CameraId.New, Camera.lookAt (rand.UniformV3dDirection() * (2.0 + rand.UniformDouble() * 6.0)) (rand.UniformV3dDirection()) (rand.UniformV3dDirection())
            )

        let tracks = points |> Array.map (fun _ -> TrackId.New)

        let tracks2 = points |> Array.map (fun _ -> TrackId.New)

        let observe (c : Camera) =
            points |> Seq.mapi (fun i p ->
                let tid = tracks.[i]
                let c = Camera.project1 c p
                tid, c
            ) |> MapExt.ofSeq

        let observe2 (c : Camera) =
            points |> Seq.mapi (fun i p ->
                let tid = tracks2.[i]
                let c = Camera.project1 c p
                tid, c
            ) |> MapExt.ofSeq

        let cams = cameras |> List.map ( fun (cid, c) -> cid, observe c ) 
        let cams2 = cameras2 |> List.map ( fun (cid, c) -> cid, observe2 c ) 
            
        let nets = cams @ cams2 |> PhotoNetwork.ofList PhotoNetworkConfig.Default

        Log.start("solved")
        for net in nets do
            Log.line "%d: %.6f" net.count net.cost 
        Log.stop()

    ()


let arrarr (arr : 'a[][]) =
    Array2D.init (arr |> Array.length) (arr.[0] |> Array.length) ( fun i j -> arr.[i].[j] )
       

let pointCloudTrafo (l : MapExt<TrackId, V3d>) (r : MapExt<TrackId, V3d>) =
    
    let pairs = MapExt.intersect l r |> MapExt.values |> Seq.toArray
    if pairs.Length < 3 then
        Trafo3d.Identity
    else
        
        // A * x = b

        // M * (p,1) = q

        // dot (p1,1) M.R0 = q1.X 
        // dot (p2,1) M.R1 = q2.Y
        // dot (p2,1) M.R2 = q2.Z
         

        let solveRow (ri : int) =
            let M0 = 
                arrarr [|
                    for i in 0 .. pairs.Length-1 do
                        let (pi, qi) = pairs.[i]
                        yield [| pi.X; pi.Y; pi.Z; 1.0 |]    
                |]

            let b = 
                [| 
                    for i in 0 .. pairs.Length-1 do 
                        let (pi, qi) = pairs.[i]
                        yield qi.[ri]
                |]

            let perm = M0.QrFactorize()
            M0.QrSolve(perm, b) |> V4d

        let r0 = solveRow 0
        let r1 = solveRow 1
        let r2 = solveRow 2

        let mat = M44d.FromRows(r0, r1, r2, V4d.OOOI)
        Trafo3d(mat, mat.Inverse)







    


module Sg =
            
    let hsv2rgb (h : float) (s : float) (v : float) =
        let s = clamp 0.0 1.0 s
        let v = clamp 0.0 1.0 v

        let h = h % 1.0
        let h = if h < 0.0 then h + 1.0 else h
        let hi = floor ( h * 6.0 ) |> int
        let f = h * 6.0 - float hi
        let p = v * (1.0 - s)
        let q = v * (1.0 - s * f)
        let t = v * (1.0 - s * ( 1.0 - f ))
        match hi with
            | 1 -> V3d(q,v,p)
            | 2 -> V3d(p,v,t)
            | 3 -> V3d(p,q,v)
            | 4 -> V3d(t,p,v)
            | 5 -> V3d(v,p,q)
            | _ -> V3d(v,t,p)

    let inline color< ^a when ^a : (member Id : int) > (tid : ^a) =
        let id = (^a : (member Id : int) (tid))
        let c = hsv2rgb (float id * (1.0/20.0)) 1.0 0.5
        C4f(c.X, c.Y, c.Z).ToC4b()

    let camera (far : float) (cid : CameraId) (c : Camera) =
        let frust = Box3d(-V3d.III, V3d.III)
        

        let projectionTrafo = Frustum.perspective 90.0 0.001 far 1.0 |> Frustum.projTrafo

        let trafo = (c |> Camera.viewTrafo) * projectionTrafo

        Sg.wireBox' (color cid) frust
            |> Sg.transform trafo.Inverse
            
    let cameras (far : float) (cams : MapExt<CameraId,Camera>) =
        cams |> MapExt.toSeq |> Seq.map (fun (id,c) -> camera far id c) |> Sg.ofSeq

    let points ( ps : MapExt<TrackId, V3d> ) =
        let points = 
            ps |> MapExt.toSeq |> Seq.map (snd >> V3f)
               |> Seq.toArray 
        let cols = 
            ps |> MapExt.toSeq |> Seq.map (fst >> color)
               |> Seq.toArray

        Sg.render IndexedGeometryMode.PointList (DrawCallInfo( FaceVertexCount = points.Length, InstanceCount = 1 ))
            |> Sg.vertexAttribute' DefaultSemantic.Positions points
            |> Sg.vertexAttribute' DefaultSemantic.Colors cols

    let private overlayPass = RenderPass.after "overlay" RenderPassOrder.Arbitrary RenderPass.main
    let private consolas = new Font("Consolas")
    let photonetworkAnnotations (net : PhotoNetwork) =
        let edges = 
            let lines = 
                net.usedEdges 
                    |> Seq.collect (fun (l,r) -> [ net.cameras.[l].location; net.cameras.[r].location ])
                    |> Seq.map V3f
                    |> Seq.toArray

            let colors = 
                net.usedEdges 
                    |> Seq.collect (fun (l,r) -> [ C4b.Blue; C4b.Green ])
                    |> Seq.toArray
                
            Sg.render IndexedGeometryMode.LineList (DrawCallInfo( FaceVertexCount = lines.Length, InstanceCount = 1 ))
                |> Sg.vertexAttribute' DefaultSemantic.Positions lines
                |> Sg.vertexAttribute' DefaultSemantic.Colors colors

        let edgeAnnotations = 
            net.usedEdges 
                |> Seq.map (fun (l,r) ->
                    let edge = net.edges.[(l,r)]
                    let pl = net.cameras.[l].location 
                    let pr = net.cameras.[r].location 
                    let c =  0.5 * (pl + pr)
                    
                    let text = 
                        String.concat "\n" [
                            sprintf "count:  %d" edge.leftObservations.Count
                            sprintf "length: %.2f" (Vec.length (pr - pl))
                            sprintf "inverse: %A" edge.inverse
                        ]

                    let shapes = consolas.Layout(C4b.Black, TextAlignment.Center, Box2d(-V2d.II, V2d.II), text)
                    Sg.shapeWithBackground (C4b(255uy,255uy,255uy,200uy)) (Mod.constant shapes)
                        |> Sg.scale 0.1
                        |> Sg.billboard
                        |> Sg.transform (Trafo3d.Translation(c))
                )
                |> Sg.ofSeq
                |> Sg.blendMode (Mod.constant BlendMode.Blend)
                |> Sg.pass overlayPass

        Sg.ofList [
            edges
            edgeAnnotations
        ]

    let photonetworks (far : float) (nets : list<PhotoNetwork>)=
        nets |> List.mapi ( fun i net ->
            Sg.ofList [
                net.cameras |> cameras far
                net.points |> points

                photonetworkAnnotations net

            ]   
            |> Sg.andAlso (IndexedGeometryPrimitives.coordinateCross V3d.III |> Sg.ofIndexedGeometry)
            |> Sg.translate (float i * 10.0) 0.0 0.0

        )   
        |> Sg.ofList
        |> Sg.shader {
            do! DefaultSurfaces.trafo
            do! DefaultSurfaces.vertexColor
        }

           
           


let renderNetwork () =
    let points =
        Array.init 1000 (fun _ ->
            rand.UniformV3dDirection() * (rand.UniformDouble() * 2.0)
        )
        
    let cameras = 
        List.init 5 ( fun i ->
            //if i = 0 then CameraId.New, Camera.lookAt (V3d(0,-5,0)) V3d.Zero V3d.OOI
            //else
                CameraId.New, Camera.lookAt (rand.UniformV3dDirection() * (2.0 + rand.UniformDouble() * 6.0)) (rand.UniformV3dDirection()) (rand.UniformV3dDirection())
        )
            

    let tracks = points |> Array.map (fun _ -> TrackId.New)
    
    let original =
        {
            config              = PhotoNetworkConfig.Default
            count               = cameras.Length
            cost                = 0.0
            cameras             = MapExt.ofList cameras
            edges               = MapExt.empty
            usedEdges           = Set.empty
            points              = Array.zip tracks points |> MapExt.ofArray
            observations        = MapExt.empty
        }

    let rand = RandomSystem()
    let jiggleRadius = 0.01
    let mismatchChance = 0.03 //1
    let observeChance = 0.4

    let bounds = Box2d(-V2d.II, V2d.II)
    let mutable mismatchCount = 0

    let observe (c : Camera) =
        points |> Seq.mapi (fun i p ->
            let tid = tracks.[i]
            if rand.UniformDouble() <= mismatchChance then
                mismatchCount <- mismatchCount + 1
                tid, rand.UniformV2d(bounds)
            else
                let c = Camera.project1 c p

                let c = 
                    if jiggleRadius > 0.0 then
                        c + rand.UniformV2dDirection() * rand.UniformDouble() * jiggleRadius
                    else
                        c

                tid, c
        )
        |> Seq.filter (fun _ -> rand.UniformDouble() <= observeChance)
        |> MapExt.ofSeq

    let cams = cameras |> List.map ( fun (cid, c) -> cid, observe c ) 
            
    let nets = cams |> PhotoNetwork.ofList PhotoNetworkConfig.Default
    
    let trafo =
        match nets with
            | net :: _ ->
                pointCloudTrafo net.points original.points
            | _ ->
                Trafo3d.Identity


    let sg = 
        [ Sg.photonetworks 1.0 (nets |> List.map (PhotoNetwork.transformed trafo))
          Sg.photonetworks 0.7 [original] //|> Sg.translate 0.0 10.0 0.0
        ] |> Sg.ofList
            
    let font = Font("Consolas")
    let overlay =
        let content =
            String.concat "\r\n" [
                sprintf "mismatches: %d" mismatchCount
                sprintf "points: %d" (nets |> List.sumBy (fun n -> n.points.Count))
            ]
        Sg.text font C4b.White (Mod.constant content)
            |> Sg.billboard
            |> Sg.translate 0.0 0.0 5.0

    let sg = Sg.ofList [sg]

    use app = new OpenGlApplication()
    use win = app.CreateSimpleRenderWindow(8)
        

    let proj = win.Sizes    |> Mod.map (fun s -> Frustum.perspective 60.0 0.1 10000.0 (float s.X / float s.Y))
                            |> Mod.map Frustum.projTrafo

    let view = CameraView.lookAt (V3d(20.0, 0.0, 0.0)) V3d.Zero V3d.OOI
                        |> DefaultCameraController.controlWithSpeed (Mod.init 2.5) win.Mouse win.Keyboard win.Time
                        |> Mod.map CameraView.viewTrafo

    let sg = sg  |> Sg.viewTrafo view |> Sg.projTrafo proj

    let task = app.Runtime.CompileRender(win.FramebufferSignature, sg)

    win.RenderTask <- task

    win.Run()


[<EntryPoint>]
let main argv = 
    Ag.initialize()
    Aardvark.Init()
    Log.error "%A" System.Environment.CurrentDirectory

    renderNetwork()

    0
