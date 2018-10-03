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
                V2d(-0.124,0.234674)
                V2d(-0.364,-0.3474)
            |]

        let d = 
            [|
                1.2
                1.4
                1.23
                1.346
                1.7522
                1.356
                1.5
                1.5467
            |]

        let cam = 
            { 
                view = CameraView.lookAt (V3d(22.0,-42.3,-41.5)) (V3d(200.0, 100.0, 300.0)) (V3d(2.3,-2.1,-1.1).Normalized)
                proj = {aspect = 2.123; focalLength = 3.125125; principalPoint =(V2d(-0.42,0.11))} 
            }

        let (image,world) = 
            a |> Array.mapi ( fun i v ->
                v, (
                    let ray = Camera.unproject (v) cam
                    ray.GetPointOnRay d.[i]
                )
            )   |> Array.take 8
                |> Array.unzip

        let intern = (Projection.toTrafo cam.proj).Forward

        match OpenCV.solvePnP SolverKind.Iterative image world intern (Array.replicate 4 0.0) with
        | None -> Log.line "fail"
        | Some e ->
            
            let cv = 
                { 
                    trafo = e
                }

            let loc = cv.Location
            let fw = cv.Forward
            let up = cv.Up
            let ri = cv.Right

            let good = CameraView.approxEqual 1E-4 cv cam.view
             
            Log.line "%A, %A %A" fw loc good


        0 // return an integer exit code
