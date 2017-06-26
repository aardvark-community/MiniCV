namespace Aardvark.Reconstruction

open System
open Aardvark.Base
open System.Threading

module Seq =
    let choosei (f : int -> 'a -> Option<'b>) (s : seq<'a>) = s |> Seq.mapi f |> Seq.choose id

module List =
    let choosei (f : int -> 'a -> Option<'b>) (s : list<'a>) = s |> List.mapi f |> List.choose id


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
