namespace MiniCV

open System
open Aardvark.Base
open Aardvark.Base.Sorting

open System.Collections.Generic
open System.Runtime.InteropServices
open System.Security
open MiniCV
open Microsoft.FSharp.NativeInterop

#nowarn "9"
#nowarn "51"

[<StructLayout(LayoutKind.Sequential)>]
type RecoverPoseConfig  =
    struct 
            
        val mutable public FocalLength      : double
        val mutable public PrincipalPoint   : V2d
        val mutable public Probability      : double
        val mutable public InlierThreshold  : double
            
        new(f : double, pp : V2d, p : double, t : double) = 
            {
                FocalLength = f
                PrincipalPoint = pp
                Probability = p
                InlierThreshold = t
            }

        static member Default =
            RecoverPoseConfig(1.0, V2d.Zero, 0.999, 0.005)

    end

[<Struct>]
type KeyPoint(index : int, pos : V2d, size : float, angle : float, response : float, octave : int, descriptorDimension : int, descriptors : uint8[]) =
    member x.Index = index
    member x.Position = pos
    member x.Size = size
    member x.Angle = angle
    member x.Response = response
    member x.Octave = int

    member x.Descriptor =
        let fst = int64 descriptorDimension * int64 index
        Vector<uint8>(descriptors, fst, int64 descriptorDimension, 1L)
        
    override x.ToString() =
        sprintf "{ pos = %A; size = %A; angle = %A }" pos size angle

type ImageFeatures =
    {
        points                  : KeyPoint[]
        descriptors             : uint8[]
        descriptorDimension     : int
    }

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module ImageFeatures =
    let private cmp (d : int) = Func<Vector<'a>, Vector<'a>, int>(fun (l : Vector<'a>) (r : Vector<'a>) -> compare l.[d % int l.S] r.[d % int r.S])
    
    let rec private pointKdTree (perm : int[]) (data : Vector<'a>[]) (l : int) (r : int) (d : int)=
        if r > l then
            let m = (r - l) / 2
            perm.PermutationQuickMedian(data, cmp d, l, r, m)
            pointKdTree perm data l (l + m) (d + 1)
            pointKdTree perm data (l + m + 1) r (d + 1)

    type private ClosestPoints =
        {
            count : int
            matches : MapExt<float, list<int>>
        }

        member x.Count = x.count
        member x.Min = x.matches.TryMinKey |> Option.defaultValue Double.NegativeInfinity
        member x.Max = x.matches.TryMaxKey |> Option.defaultValue Double.PositiveInfinity

    [<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
    module private ClosestPoints =
        let empty = { count = 0; matches = MapExt.empty }

        let add (n : int) (dist : float) (id : int) (c : ClosestPoints) =
            if c.count = n then
                let mm =
                    MapExt.alter dist (fun old ->
                        let old = Option.defaultValue [] old
                        Some (id :: old)
                    ) c.matches

                let max = mm.TryMaxKey |> Option.get
                { 
                    count = n
                    matches = 
                        mm |> MapExt.alter max (fun o -> 
                            match o with
                                | Some [_] -> None
                                | Some (_ :: rest) -> Some rest
                                | _ -> None
                        )
                }
            else
                { 
                    count = c.count + 1
                    matches = 
                        MapExt.alter dist (fun old ->
                            let old = Option.defaultValue [] old
                            Some (id :: old)
                        ) c.matches
                }
           

        let union (l : ClosestPoints) (r : ClosestPoints) =
            { count = l.count + r.count; matches = MapExt.unionWith List.append l.matches r.matches }

    let rec private nearest (current : ClosestPoints) (n : int) (maxDist : float) (sub : 'a -> 'a -> float) (perm : int[]) (data : Vector<'a>[]) (l : int) (r : int) (d : int) (v : Vector<'a>) : ClosestPoints =
        let dist (l : Vector<'a>) (r : Vector<'a>) =
            let mutable sum = 0.0
            for i in 0 .. int l.Size - 1 do
                let v = sub r.[i] l.[i]
                sum <- sum + v * v
            sqrt sum

        if r > l then
            let m = (r - l) / 2
            let mm = data.[perm.[l + m]]
            let dimDist = sub v.[d % int v.Size] mm.[d % int mm.Size]
            let dist = dist v mm

            let current =
                if dist < maxDist then ClosestPoints.add n dist perm.[m] current
                else current

            let maxDist = 
                if current.Count >= n then current.Max
                else maxDist

            if dimDist > maxDist then
                nearest current n maxDist sub perm data (l + m + 1) r (d + 1) v

            elif dimDist < -maxDist then
                nearest current n maxDist sub perm data l (l + m) (d + 1) v

            elif dimDist >= 0.0 then
                let rv = nearest current n maxDist sub perm data (l + m + 1) r (d + 1) v
                let lMax = if rv.Count >= n then rv.Max else maxDist
                let lv = nearest rv n lMax sub perm data l (l + m) (d + 1) v
                ClosestPoints.union lv rv

            else
                let lv = nearest current n maxDist sub perm data l (l + m) (d + 1) v
                let rMax = if lv.Count >= n then lv.Max else maxDist
                let rv = nearest lv n rMax sub perm data (l + m + 1) r (d + 1) v
                ClosestPoints.union lv rv

        else
            current

    let matches (l : ImageFeatures) (r : ImageFeatures) =
        let rf = r.points |> Array.map (fun pt -> pt.Descriptor)
        let perm = Array.init rf.Length id
        pointKdTree perm rf 0 rf.Length 0

        let sub (l : byte) (r : byte) =
            (float l - float r) / 255.0

        l.points |> Array.choosei (fun li lf ->
            let closest = nearest ClosestPoints.empty 2 Double.PositiveInfinity sub perm rf 0 rf.Length 0 lf.Descriptor
            let closest = 
                closest.matches 
                    |> MapExt.toSeq 
                    |> Seq.collect (fun (d,s) -> s |> Seq.map (fun ri -> li, ri, d)) 
                    |> Seq.atMost 2
                    |> Seq.toList
            match closest with
                | [] -> None
                | c -> Some c
        )




module OpenCV =

    [<StructLayout(LayoutKind.Sequential)>]
    type KeyPoint2d =
        struct
            val mutable public pt : V2f
            val mutable public size : float32
            val mutable public angle : float32
            val mutable public response : float32
            val mutable public octave : int
            val mutable public class_id : int


        end

    [<StructLayout(LayoutKind.Sequential)>]
    type DetectorResult =
        struct
            val mutable public PointCount : int
            val mutable public DescriptorEntries : int
            val mutable public Points : nativeptr<KeyPoint2d>
            val mutable public Descriptors : nativeptr<uint8>
        end

    type DetectorMode =
        | Akaze = 1
        | Orb = 2
        | Brisk = 3


    module Native =

        [<Literal>]
        let lib = @"D:\proj_2\minicv-besser\bin\Debug\MiniCVNative.dll"
        
        [<DllImport(lib, EntryPoint = "cvRecoverPose"); SuppressUnmanagedCodeSecurity>]
        extern int cvRecoverPose_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat, V3d& tVec, byte[] ms)
        
        [<DllImport(lib, EntryPoint = "cvRecoverPoses"); SuppressUnmanagedCodeSecurity>]
        extern bool cvRecoverPoses_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat1, M33d& rMat2, V3d& tVec, byte[] ms)
        
        [<DllImport(lib, EntryPoint = "cvDetectFeatures"); SuppressUnmanagedCodeSecurity>]
        extern DetectorResult* cvDetectFeatures_(byte* data, int width, int height, int channels, DetectorMode mode)
        
        [<DllImport(lib, EntryPoint = "cvFreeFeatures"); SuppressUnmanagedCodeSecurity>]
        extern void cvFreeFeatures_(DetectorResult* res)

        [<DllImport(lib, EntryPoint = "cvDoStuff"); SuppressUnmanagedCodeSecurity>]
        extern void cvDoStuff_()

        [<DllImport(lib, EntryPoint = "cvTest"); SuppressUnmanagedCodeSecurity>]
        extern void cvTest()

        [<DllImport(lib, EntryPoint = "cvFivePoint"); SuppressUnmanagedCodeSecurity>]
        extern int cvFivePoint(V2d[] pa, V2d[] pb, M33d* Es)

        [<DllImport(lib, EntryPoint = "cvSolvePnP"); SuppressUnmanagedCodeSecurity>]
        extern bool cvSolvePnP(V2d[] imgPoints, V3d[] worldPoints, int N, M33d intern, float[] distortion, int solverKind, V3d& t, V3d& r)
        
    let private copy (src : nativeptr<'a>) (dst : 'a[]) (cnt : int) =
        let gc = GCHandle.Alloc(dst, GCHandleType.Pinned)
        try
            Marshal.Copy(NativePtr.toNativeInt src, gc.AddrOfPinnedObject(), nativeint sizeof<'a> * nativeint cnt)
        finally
            gc.Free()

    let detectFeatures (mode : DetectorMode) (img : PixImage<byte>) =
        let img = img.ToCanonicalDenseLayout() |> unbox<PixImage<byte>>

        let gc = GCHandle.Alloc(img.Volume.Data, GCHandleType.Pinned)
        let mutable ptr = NativePtr.zero
        try
            ptr <- Native.cvDetectFeatures_(NativePtr.ofNativeInt (gc.AddrOfPinnedObject()), img.Size.X, img.Size.Y, img.ChannelCount, mode)
            let v = NativePtr.read ptr
            
            if v.PointCount = 0 then
                {
                    points                  = [||]
                    descriptors             = [||]
                    descriptorDimension     = 61
                }
            else
                let pts : KeyPoint2d[] = Array.zeroCreate v.PointCount
                let descriptors : uint8[] = Array.zeroCreate v.DescriptorEntries

                copy v.Points pts pts.Length
                copy v.Descriptors descriptors descriptors.Length

                let dim = descriptors.Length / pts.Length

                let points = pts |> Array.mapi (fun i pt -> KeyPoint(i, V2d pt.pt, float pt.size, float pt.angle, float pt.response, pt.octave, dim, descriptors))
                {
                    points                  = points
                    descriptors             = descriptors
                    descriptorDimension     = dim
                }
        finally
            Native.cvFreeFeatures_ ptr
            gc.Free()

    let recoverPose (cfg : RecoverPoseConfig) (a : V2d[]) (b : V2d[]) =
        let mutable m = M33d.Identity
        let mutable t = V3d(100,123,432)
        let mutable cfg = cfg
        let mutable ms = Array.create a.Length 0uy
        let res = Native.cvRecoverPose_(&&cfg, a.Length, a, b, &m, &t, ms)
        (res, m, t, ms)

    let recoverPoses (cfg : RecoverPoseConfig) (a : V2d[]) (b : V2d[]) =
        let mutable m1 = M33d.Identity
        let mutable m2 = M33d.Identity
        let mutable t = V3d(100,123,432)
        let mutable cfg = cfg
        let mutable ms = Array.create a.Length 0uy
        Native.cvRecoverPoses_(&&cfg, a.Length, a, b, &m1, &m2, &t, ms) |> ignore
        (m1, m2, t, ms)

    let recoverPoses2 (cfg : RecoverPoseConfig) (a : V2d[]) (b : V2d[]) =
        let scale = 2.0
        let offset = V2d(0.0,0.0)
        let a = a |> Array.map ( fun v -> V2d(0.5 * v.X + offset.X, offset.Y - 0.5*v.Y) * scale )
        let b = b |> Array.map ( fun v -> V2d(0.5 * v.X + offset.X, offset.Y - 0.5*v.Y) * scale )
        
        let mutable m1 = M33d.Identity
        let mutable m2 = M33d.Identity
        let mutable t = V3d(100,123,432)
        let mutable cfg = cfg
        cfg.FocalLength <- scale * cfg.FocalLength / 2.0
        cfg.PrincipalPoint <- scale * offset
        cfg.InlierThreshold <- scale * 0.5 * cfg.InlierThreshold
        let mutable ms = Array.create a.Length 0uy
        Native.cvRecoverPoses_(&&cfg, a.Length, a, b, &m1, &m2, &t, ms) |> ignore

        let m1 = m1.Transposed
        let m2 = m2.Transposed

        let C = M33d.FromCols(V3d.IOO, -V3d.OIO, -V3d.OOI)
        let m1 = C * m1 * C
        let m2 = C * m2 * C

        let t = C * t 

        let possiblePoses =
            if m1 = m2 then
                [CameraPose(0, 1, m1, t, false)]
            else
                [
                    CameraPose(0, 1, m1, t, false)
                    CameraPose(1, 1, m2, t, false)
                ]

        let mask =
            Array.map ((<>) 0uy) ms

        possiblePoses, mask


    let fivepoint (a : V2d[]) (b : V2d[]) : M33d[] =
        let res = Array.zeroCreate 10
        use ptr = fixed res
        let cnt = Native.cvFivePoint(a,b,ptr)
        
        if cnt > 0 then
            Array.take cnt res
        else
            [||]

    type SolverKind =
    | EPNP
    | P3P
    | AP3P
    | Iterative
    
    let solvePnP (solver : SolverKind) (imgPoints : V2d[]) (worldPoints : V3d[]) (intern : M33d) (distortionCoeffs : float[]) =
        if imgPoints.Length <> worldPoints.Length then
            None
        else
            let kind =
                match solver with
                | Iterative -> 0
                | EPNP -> 1
                | P3P -> 2
                | AP3P -> 5
        
            let mutable tRes = Unchecked.defaultof<_>
            let mutable rRes = Unchecked.defaultof<_>
            
            if Native.cvSolvePnP(imgPoints, worldPoints, worldPoints.Length, intern, distortionCoeffs, kind, &tRes, &rRes) then
                let r = -rRes
                let t = -tRes
                let ang = r.Length
                let axs = r.Normalized
                let rot = Rot3d(axs,ang)
                let trn = rot.TransformPos t
                let e = Euclidean3d(rot,trn)
                
                Some e
            else
                None

    open System
    open System.IO
    open System.Text

    let undistortImages () =
        Native.cvDoStuff_()
            