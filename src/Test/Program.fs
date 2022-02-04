open System
open System.IO
open System.Net.Http
open Aardvark.Base
open MiniCV


[<EntryPoint>]
let main argv =
    Aardvark.Init()

    use c = new HttpClient()
    let data = c.GetByteArrayAsync("https://images.pexels.com/photos/245535/pexels-photo-245535.jpeg?cs=srgb&dl=pexels-snapwire-245535.jpg&fm=jpg").Result
    let img = 
        use ms = new MemoryStream(data)
        PixImageSharp.Create(ms).ToPixImage<byte>(Col.Format.RGBA)

    let ftrs = MiniCV.OpenCV.detectFeatures MiniCV.OpenCV.DetectorMode.Akaze img
    printfn "%A" ftrs.points.Length
    0 // return an integer exit code
