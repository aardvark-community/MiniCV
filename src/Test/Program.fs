namespace MiniCVTest

// Learn more about F# at http://fsharp.org

open System
open MiniCV
open Aardvark.Base

[<AutoOpen>]
module fufuf =
    open MiniCV.OpenCV
    
    let inline printMat (name : string) (m : IMatrix< ^a >) =
        let table = 
            [
                for y in 0L .. m.Dim.Y - 1L do
                    yield [
                        for x in 0L .. m.Dim.X - 1L do
                            let str = sprintf "%.3f" m.[V2l(x,y)]
                            if str.StartsWith "-" then yield str
                            else yield " " + str
                    ]
            ]

        let maxLength = table |> Seq.collect (fun row -> row |> Seq.map (fun s -> s.Length)) |> Seq.max
        let pad (str : string) =
            if str.Length < maxLength then 
                let m = maxLength - str.Length
                let b = m / 2
                let a = m - b

                String(' ', b) + str + String(' ', a)
            else    
                str

        let rows = 
            table |> List.map (fun row ->
                row |> List.map pad |> String.concat "  "
            )
        printfn "%s (%dx%d)" name m.Dim.Y m.Dim.X
        for r in rows do
            printfn "   %s" r

    let gramSchmidt (m : M33d) =
        let x = Vec.normalize m.C0
        let y = Vec.normalize m.C1
        let z = Vec.normalize m.C2

        let y1 = y - Vec.dot z y * z |> Vec.normalize
        let x1 = x - Vec.dot z x * z - Vec.dot y1 x * y1 |> Vec.normalize

        M33d.FromCols(x1,y1,z)
    [<EntryPoint>]
    let main argv =
        printfn "Hello World from F#!"


        let a = 
            [|
                V2d.Zero
                V2d.IO
                V2d.OI
                V2d(-0.2360,0.25)
                V2d(-0.6792,-0.854)
                V2d(0.74,0.73)
                V2d(-0.33,-0.256)
                V2d(0.19,0.251)
                V2d(0.74,-0.24)
                V2d(-0.124,0.234674)
                V2d(-0.364,-0.3474)
            |]

        let d = 
            [|
                1.5
                1.7
                1.2
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
                view = CameraView.lookAt V3d.IOO V3d.OOO V3d.OOI//(V3d(22.0,-42.3,-41.5)) (V3d(200.0, 100.0, 300.0)) (V3d(2.3,-2.1,-1.1).Normalized)
                proj = Projection.identity//{aspect = 2.123; focalLength = 3.125125; principalPoint =(V2d(-0.42,0.11))} 
            }

        let (image,world) = 
            a |> Array.mapi ( fun i v ->
                v, (
                    let ray = Camera.unproject (v) cam
                    ray.GetPointOnRay d.[i]
                )
            )   |> Array.take 3
                |> Array.unzip

        let intern = (Projection.toTrafo cam.proj).Forward

        let ms = OpenCV.solveAp3p image world intern
        
        printMat "original" ((cam.view |> CameraView.viewTrafo).Forward.UpperLeftM33())

        ms |> Array.iteri ( fun i (r,t) -> 
            let r = (gramSchmidt r)
            printMat (sprintf "%d" i) r

            //let R = 
            //    Rot3d.FromM33d(gramSchmidt r)

            //let t = t

            //let e = Euclidean3d(R,t)

            //let cv = 
            //    { 
            //        trafo = e
            //    }
        
            //let loc = cv.Location
            //let fw = cv.Forward
            //let up = cv.Up
            //let ri = cv.Right
        
            //let good = CameraView.approxEqual 1E-4 cv cam.view
        
            //Log.line "%A, %A %A" fw loc good
        )
        printfn "%A" 123562356

        //match OpenCV.solvePnP SolverKind.Iterative image world intern (Array.replicate 4 0.0) with
        //| None -> Log.line "fail"
        //| Some e ->
            
        //    let cv = 
        //        { 
        //            trafo = e
        //        }

        //    let loc = cv.Location
        //    let fw = cv.Forward
        //    let up = cv.Up
        //    let ri = cv.Right

        //    let good = CameraView.approxEqual 1E-4 cv cam.view
             
        //    Log.line "%A, %A %A" fw loc good


        0 // return an integer exit code
