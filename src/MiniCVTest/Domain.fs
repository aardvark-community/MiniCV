namespace Aardvark.Reconstruction

open Aardvark.Base
open Aardvark.Reconstruction
open Aardvark.Base.Incremental

[<DomainType>]
type Model =
    {
        cameras : array<CameraId * Camera>
        points  : array<TrackId * V3d>
        edges   : array<CameraId * CameraId * PhotoNetworkEdge>
    }