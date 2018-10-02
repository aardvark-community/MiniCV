namespace MiniCVTest

// Learn more about F# at http://fsharp.org

open System
open MiniCV
open Aardvark.Base

[<AutoOpen>]
module fufuf =
    open MiniCV.OpenCV

    [<EntryPoint>]
    let main argv =
        printfn "Hello World from F#!"


        let a = 
            [|
                V2d(-0.10,0.25)
                V2d(-0.42,-0.254)
                V2d(0.44,0.23)
                V2d(-0.33,-0.256)
                V2d(0.19,0.251)
                V2d(0.74,-0.24)
            |]

        let d = 
            [|
                1.2
                1.4
                1.23
                1.346
                1.7522
                1.356
            |]

        let cam = { view = CameraView.lookAt V3d.III V3d.OOO V3d.OOI; proj = {aspect = 4.0/3.0; focalLength = 1.5423; principalPoint = (V2d(-0.0042,0.0011))} }

        let (image,world) = 
            a |> Array.mapi ( fun i v ->
                v, (
                    let ray = Camera.unproject v cam
                    ray.GetPointOnRay d.[i]
                )
            )   |> Array.take 6
                |> Array.unzip

        let intern = (Projection.toTrafo cam.proj).Forward

        match OpenCV.solvePnP SolverKind.Iterative image world intern (Array.replicate 4 0.0) with
        | None -> Log.line "fail"
        | Some e ->
            
            let loc = e.TransformPos V3d.OOO
            let fw = e.TransformDir -V3d.OOI

            Log.line "%A, %A" fw loc


        0 // return an integer exit code
