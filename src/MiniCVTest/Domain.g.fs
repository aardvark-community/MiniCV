namespace Aardvark.Reconstruction

open System
open Aardvark.Base
open Aardvark.Base.Incremental
open Aardvark.Reconstruction

[<AutoOpen>]
module Mutable =

    
    
    type MModel(__initial : Aardvark.Reconstruction.Model) =
        inherit obj()
        let mutable __current = __initial
        let _cameras = ResetMod.Create(__initial.cameras)
        let _points = ResetMod.Create(__initial.points)
        let _edges = ResetMod.Create(__initial.edges)
        
        member x.cameras = _cameras :> IMod<_>
        member x.points = _points :> IMod<_>
        member x.edges = _edges :> IMod<_>
        
        member x.Update(v : Aardvark.Reconstruction.Model) =
            if not (System.Object.ReferenceEquals(__current, v)) then
                __current <- v
                
                ResetMod.Update(_cameras,v.cameras)
                ResetMod.Update(_points,v.points)
                ResetMod.Update(_edges,v.edges)
                
        
        static member Create(__initial : Aardvark.Reconstruction.Model) : MModel = MModel(__initial)
        static member Update(m : MModel, v : Aardvark.Reconstruction.Model) = m.Update(v)
        
        override x.ToString() = __current.ToString()
        member x.AsString = sprintf "%A" __current
        interface IUpdatable<Aardvark.Reconstruction.Model> with
            member x.Update v = x.Update v
    
    
    
    [<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
    module Model =
        [<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
        module Lens =
            let cameras =
                { new Lens<Aardvark.Reconstruction.Model, Microsoft.FSharp.Core.array<(Aardvark.Reconstruction.CameraId * Aardvark.Reconstruction.Camera)>>() with
                    override x.Get(r) = r.cameras
                    override x.Set(r,v) = { r with cameras = v }
                    override x.Update(r,f) = { r with cameras = f r.cameras }
                }
            let points =
                { new Lens<Aardvark.Reconstruction.Model, Microsoft.FSharp.Core.array<(Aardvark.Reconstruction.TrackId * Aardvark.Base.V3d)>>() with
                    override x.Get(r) = r.points
                    override x.Set(r,v) = { r with points = v }
                    override x.Update(r,f) = { r with points = f r.points }
                }
            let edges =
                { new Lens<Aardvark.Reconstruction.Model, Microsoft.FSharp.Core.array<(Aardvark.Reconstruction.CameraId * Aardvark.Reconstruction.CameraId * Aardvark.Reconstruction.PhotoNetworkEdge)>>() with
                    override x.Get(r) = r.edges
                    override x.Set(r,v) = { r with edges = v }
                    override x.Update(r,f) = { r with edges = f r.edges }
                }
