open System
open System.IO
open System.Net.Http
open Aardvark.Base
open MiniCV
open MiniCVTest

let testPnp() =
    let rand = RandomSystem()
    let noisePercent = 0.5
    let noise = 0.1
    let smallnoise = 0.001
    
    let pts = Array.init 20000 (fun _ -> rand.UniformV3d(Box3d.FromCenterAndSize(V3d.OOO,V3d.III*6.0)))
    let realCam = {view = CameraView.lookAt (V3d(4.0,5.0,6.0)) V3d.OOO V3d.OOI; proj = Projection.identity}
    let realObs =
        pts |> Array.map (fun p -> (Camera.projectUnsafe p realCam))
    let obs = 
        realObs |> Array.map (fun pp -> 
            if rand.UniformDouble() < noisePercent then pp + (rand.UniformV2d(Box2d.FromCenterAndSize(V2d.OO,V2d(noise,noise))))
            else pp + (rand.UniformV2d(Box2d.FromCenterAndSize(V2d.OO,V2d(smallnoise,smallnoise))))
        )
    let intern = (Projection.toTrafo realCam.proj).Forward
    let img = obs |> Array.map (fun i -> V2d(i.X, -i.Y))
    
    let ransacCfg = OpenCV.RansacParams.MaxReprojError 0.002
    let resPnP                = OpenCV.solvePnP OpenCV.SolverKind.EPNP img pts intern (Array.zeroCreate 20) 
    let resPnPRefineLM        = OpenCV.solvePnPWithRefine OpenCV.SolverKind.EPNP OpenCV.RefineKind.LM img pts intern (Array.zeroCreate 20) 
    let resPnPRefineVVS       = OpenCV.solvePnPWithRefine OpenCV.SolverKind.EPNP OpenCV.RefineKind.VVS img pts intern (Array.zeroCreate 20) 
    let resPnPRansac          = OpenCV.solvePnPRansac OpenCV.SolverKind.EPNP ransacCfg img pts intern (Array.zeroCreate 20) |> Option.map fst
    let resPnPRansacRefineLM  = OpenCV.solvePnPRansacWithRefine OpenCV.SolverKind.EPNP ransacCfg OpenCV.RefineKind.LM img pts intern (Array.zeroCreate 20) |> Option.map fst
    let resPnPRansacRefineVVS = OpenCV.solvePnPRansacWithRefine OpenCV.SolverKind.EPNP ransacCfg OpenCV.RefineKind.VVS img pts intern (Array.zeroCreate 20) |> Option.map fst
    
    let rot = Rot3d.RotationX(Constant.Pi)
    let getCam (s : Euclidean3d) = {view={trafo=rot*s};proj=realCam.proj}
    let camScore (c0 : Camera) (c1 : Camera) =
        Vec.distance c0.Location c1.Location +
        Vec.distance (c0.Location+c0.Forward) (c1.Location+c1.Forward) +
        Vec.distance (c0.Location+c0.Up) (c1.Location+c1.Up) 
    let rmse (res : Option<Euclidean3d>) =
        match res with 
        | None -> 99999.0,99999.0
        | Some s ->
            let myCam = getCam s
            let myObs = pts |> Array.map (fun p -> Camera.projectUnsafe p myCam)
            let rmse = Array.map2 Vec.distanceSquared realObs myObs |> Array.average |> sqrt
            let cs = camScore myCam realCam
            rmse, cs
    
    let (rm,cs) = rmse resPnP in                    Log.line "resPnP:                   %.8e; %.4e" rm cs
    //let (rm,cs) = rmse resPnPRefineLM in            Log.line "resPnPRefineLM:           %.8e; %.4e" rm cs 
    let (rm,cs) = rmse resPnPRefineVVS in           Log.line "resPnPRefineVVS:          %.8e; %.4e" rm cs 
    let (rm,cs) = rmse resPnPRansac in              Log.line "resPnPRansac:             %.8e; %.4e" rm cs 
    //let (rm,cs) = rmse resPnPRansacRefineLM in      Log.line "resPnPRansacRefineLM:     %.8e; %.4e" rm cs 
    let (rm,cs) = rmse resPnPRansacRefineVVS in     Log.line "resPnPRansacRefineVVS:    %.8e; %.4e" rm cs 

[<EntryPoint>]
let main argv =
    Aardvark.Init()
    testPnp()
    exit 0

    use c = new HttpClient()
    let data = c.GetByteArrayAsync("https://images.pexels.com/photos/245535/pexels-photo-245535.jpeg?cs=srgb&dl=pexels-snapwire-245535.jpg&fm=jpg").Result
    let img = 
        use ms = new MemoryStream(data)
        PixImageSharp.Create(ms).ToPixImage<byte>(Col.Format.RGBA)

    printfn "SIFT"
    let ftrs = MiniCV.OpenCV.detectFeatures MiniCV.OpenCV.DetectorMode.Sift img
    printfn "  count: %A" ftrs.points.Length
    if ftrs.points.Length > 0 then 
        let p = ftrs.points.[0]
        printfn "  dim:           %A" ftrs.descriptorDimension
        printfn "  descriptor[0]: %0A" p.Descriptor
        printfn "  point:         %A" p
        
    printfn "AKAZE"
    let ftrs = MiniCV.OpenCV.detectFeatures MiniCV.OpenCV.DetectorMode.Akaze img
    printfn "  count: %A" ftrs.points.Length
    if ftrs.points.Length > 0 then 
        let p = ftrs.points.[0]
        printfn "  dim:           %A" ftrs.descriptorDimension
        printfn "  descriptor[0]: %0A" p.Descriptor
        printfn "  point:         %A" p
    0 // return an integer exit code
