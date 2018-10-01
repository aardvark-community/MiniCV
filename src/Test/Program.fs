// Learn more about F# at http://fsharp.org

open System
open MiniCV
open Aardvark.Base

[<EntryPoint>]
let main argv =
    printfn "Hello World from F#!"


    let a = 
        [|
            V2d(-1.0,2.5)
            V2d(-0.42,-2.54)
            V2d(4.4,0.23)
            V2d(-3.3,-2.56)
            V2d(1.9,2.51)
        |]

    let b = 
        [|
            V2d(-1.0,2.5)    + V2d.II
            V2d(-0.42,-2.54) + V2d.II
            V2d(4.4,0.23)    + V2d.II
            V2d(-3.3,-2.56)  + V2d.II
            V2d(1.9,2.51)    + V2d.II
        |]

    let res = OpenCV.fivepoint a b

    for r in res do
        printfn "%A" r
        printfn ""
        

    0 // return an integer exit code
