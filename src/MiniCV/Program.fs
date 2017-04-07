namespace Aardvark.Base

open System
open Aardvark.Base

open System.Runtime.InteropServices
open System.Security

module MiniCV =

    module Native =
        [<Literal>]
        let lib = "MiniCVNative"

        [<DllImport(lib, EntryPoint = "cvRecoverPose"); SuppressUnmanagedCodeSecurity>]
        extern int cvRecoverPose_(int N, V2d[] pa, V2d[] pb, M33d& rMat, V3d& tVec)

    let recoverPose (a : V2d[]) (b : V2d[]) =
        let mutable m = M33d.Identity
        let mutable t = V3d(100,123,432)
        let res = Native.cvRecoverPose_(a.Length, a, b, &m, &t)
        (res, m,t)



//[<EntryPoint>]
//let main argv = 

//    let rand = RandomSystem()

//    let pts = Array.init 100 (fun _ -> rand.UniformV3d(Box3d(-(V3d(0.1,0.1,0.1)),(V3d(0.1,0.1,0.1)))) ) 
    
//    let frustum = Frustum.perspective 90.0 1.0 1000.0 1.0 |> Frustum.projTrafo
//    let ca = CameraView.lookAt (V3d(1.0,0.0,0.0)) V3d.OOO V3d.OOI |> CameraView.viewTrafo
//    let vpa = ca * frustum
    
//    let cb = CameraView.lookAt (V3d(1.0, 0.0, 1.0)) V3d.OOI V3d.OOI |> CameraView.viewTrafo
//    let vpb = cb * frustum

//    let flip (v : V2d) = V2d(v.X, -v.Y)

//    let a = pts |> Array.map (vpa.Forward.TransformPosProj >> Vec.xy >> flip)
//    let b = pts |> Array.map (vpb.Forward.TransformPosProj >> Vec.xy >> flip)
        
//    let (inliers, R, t) = Native.recoverPose a b

//    printfn "is id: %A" (R.IsIdentity(1.0E-8))

//    printfn "%d: %A %A" inliers R t

//    printfn "trans: %A" (R * t)

//    Environment.Exit 0
    
//    Ag.initialize()
//    Aardvark.Init()



//    use app = new OpenGlApplication()
//    let win = app.CreateSimpleRenderWindow()
//    win.Text <- "Aardvark rocks \\o/"

//    let quadGeometry =
//        IndexedGeometry(
//            Mode = IndexedGeometryMode.TriangleList,
//            IndexArray = ([|0;1;2; 0;2;3|] :> Array),
//            IndexedAttributes =
//                SymDict.ofList [
//                    DefaultSemantic.Positions, [| V3f.OOO; V3f.IOO; V3f.IIO; V3f.OIO |] :> Array
//                    DefaultSemantic.Colors, [| C4b.Red; C4b.Green; C4b.Blue; C4b.Yellow |] :> Array
//                ]
//        )
       

//    let initialView = CameraView.lookAt (V3d(6,6,6)) V3d.Zero V3d.OOI
//    let view = initialView |> DefaultCameraController.control win.Mouse win.Keyboard win.Time
//    let proj = win.Sizes |> Mod.map (fun s -> Frustum.perspective 60.0 0.1 100.0 (float s.X / float s.Y))


//    let sg =
//        quadGeometry 
//            |> Sg.ofIndexedGeometry
//            |> Sg.effect [
//                DefaultSurfaces.trafo |> toEffect
//                DefaultSurfaces.vertexColor |> toEffect
//               ]
//            |> Sg.viewTrafo (view |> Mod.map CameraView.viewTrafo)
//            |> Sg.projTrafo (proj |> Mod.map Frustum.projTrafo)

    
//    let task =
//        app.Runtime.CompileRender(win.FramebufferSignature, sg)
//            |> DefaultOverlays.withStatistics

//    win.RenderTask <- task
//    win.Run()
//    0
