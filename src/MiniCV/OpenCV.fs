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
type KeyPoint(index : int, pos : V2d, size : float, angle : float, response : float, octave : int, descriptorDimension : int, descriptorType : Type, descriptors : System.Array) =
    member x.Index = index
    member x.Position = pos
    member x.Size = size
    member x.Angle = angle
    member x.Response = response
    member x.Octave = int

    member x.Descriptor =
        let res = System.Array.CreateInstance (descriptorType, descriptorDimension)
        System.Array.Copy(descriptors, descriptorDimension * index, res, 0, descriptorDimension)
        res
        
    override x.ToString() =
        sprintf "{ pos = %A; size = %A; angle = %A }" pos size angle

type ImageFeatures =
    {
        points                  : KeyPoint[]
        descriptors             : System.Array
        descriptorDimension     : int
    }

[<Struct; StructLayout(LayoutKind.Sequential)>]
type ArucoMarkerInfo =
    {
        Id : int
        P0 : V2f
        P1 : V2f
        P2 : V2f
        P3 : V2f
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

// #define CV_8U   0
// #define CV_8S   1
// #define CV_16U  2
// #define CV_16S  3
// #define CV_32S  4
// #define CV_32F  5
// #define CV_64F  6
// #define CV_16F  7
    type ElementType =
        | UInt8 = 0
        | Int8 = 1
        | UInt16 = 2
        | Int16 = 3
        | Int32 = 4
        | Float = 5
        | Double = 6
        | Half = 7

    [<StructLayout(LayoutKind.Sequential)>]
    type DetectorResult =
        struct
            val mutable public PointCount : int
            val mutable public DescriptorEntries : int
            val mutable public DescriptorElementType : ElementType
            val mutable public Points : nativeptr<KeyPoint2d>
            val mutable public Descriptors : nativeptr<uint8>
        end

    type DetectorMode =
        | Akaze = 1
        | Orb = 2
        | Brisk = 3
        | Sift = 4



    module Native =

        [<Literal>]
        let lib = @"MiniCVNative"
        
        [<DllImport(lib, EntryPoint = "cvRecoverPose"); SuppressUnmanagedCodeSecurity>]
        extern int cvRecoverPose_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat, V3d& tVec, byte[] ms)
        
        [<DllImport(lib, EntryPoint = "cvRecoverPoses"); SuppressUnmanagedCodeSecurity>]
        extern bool cvRecoverPoses_(RecoverPoseConfig* cfg, int N, V2d[] pa, V2d[] pb, M33d& rMat1, M33d& rMat2, V3d& tVec, byte[] ms)
        
        [<DllImport(lib, EntryPoint = "cvDetectFeatures"); SuppressUnmanagedCodeSecurity>]
        extern DetectorResult* cvDetectFeatures_(byte* data, int width, int height, int channels, DetectorMode mode)
        
        [<DllImport(lib, EntryPoint = "cvFreeFeatures"); SuppressUnmanagedCodeSecurity>]
        extern void cvFreeFeatures_(DetectorResult* res)

        [<DllImport(lib, EntryPoint = "cvTest"); SuppressUnmanagedCodeSecurity>]
        extern void cvTest()

        [<DllImport(lib, EntryPoint = "cvFivePoint"); SuppressUnmanagedCodeSecurity>]
        extern int cvFivePoint(V2d[] pa, V2d[] pb, M33d* Es)

        [<DllImport(lib, EntryPoint = "cvSolvePnP"); SuppressUnmanagedCodeSecurity>]
        extern bool cvSolvePnP(V2d[] imgPoints, V3d[] worldPoints, int N, M33d intern, float[] distortion, int solverKind, V3d& t, V3d& r)

        [<DllImport(lib, EntryPoint = "cvSolvePnPRansac"); SuppressUnmanagedCodeSecurity>]
        extern bool cvSolvePnPRansac(V2d[] imgPoints, V3d[] worldPoints, int N, M33d intern, float[] distortion, int solverKind, int iterationsCount, float32 reprojectionError, float confidence, V3d& t, V3d& r, int& inlierCount, int[] outInliers)
        
        [<DllImport(lib, EntryPoint = "cvRefinePnPLM"); SuppressUnmanagedCodeSecurity>]
        extern void cvRefinePnPLM(V2d[] imgPoints, V3d[] worldPoints, int N, M33d intern, float[] distortion, V3d& t, V3d& r)
        
        [<DllImport(lib, EntryPoint = "cvRefinePnPVVS"); SuppressUnmanagedCodeSecurity>]
        extern void cvRefinePnPVVS(V2d[] imgPoints, V3d[] worldPoints, int N, M33d intern, float[] distortion, V3d& t, V3d& r)
        
        [<DllImport(lib, EntryPoint = "solveAp3p"); SuppressUnmanagedCodeSecurity>]
        extern int solveAp3p(M33d* Rs, V3d* ts, float mu0, float mv0, float X0, float Y0, float Z0, float mu1, float mv1, float X1, float Y1, float Z1, float mu2, float mv2, float X2, float Y2, float Z2, float inv_fx, float inv_fy, float cx_fx, float cy_fy)
        
        
        [<DllImport(lib, EntryPoint = "cvDetectQRCode"); SuppressUnmanagedCodeSecurity>]
        extern bool cvDetectQRCode(byte* data, int width, int height, int channels, V2i* positions, int& count)


        [<DllImport(lib, EntryPoint = "cvDetectArucoMarkers"); SuppressUnmanagedCodeSecurity>]
        extern bool cvDetectArucoMarkers(byte* data, int width, int height, int channels, int& infoCount, ArucoMarkerInfo* infos)


    let solveAp3p (img : V2d[]) (world : V3d[]) (intern : M33d) =
        assert(img.Length = world.Length)
        
        let Rs : M33d[] = Array.zeroCreate 4
        let ts : V3d[]  = Array.zeroCreate 4
        
        let mu0 = img.[0].X
        let mu1 = img.[1].X
        let mu2 = img.[2].X

        let mv0 = img.[0].Y
        let mv1 = img.[1].Y
        let mv2 = img.[2].Y

        let X0 = world.[0].X
        let X1 = world.[1].X
        let X2 = world.[2].X

        let Y0 = world.[0].Y
        let Y1 = world.[1].Y
        let Y2 = world.[2].Y
        
        let Z0 = world.[0].Z
        let Z1 = world.[1].Z
        let Z2 = world.[2].Z

        let inv_fx = 1.0 / intern.M00
        let inv_fy = 1.0 / intern.M11

        let cx_fx = intern.M20 / intern.M00
        let cy_fy = intern.M21 / intern.M11

        use PRs = fixed Rs
        use Pts = fixed ts


        let cnt = Native.solveAp3p(PRs, Pts, mu0, mv0, X0, Y0, Z0, mu1, mv1, X1, Y1, Z1, mu2, mv2, X2, Y2, Z2, inv_fx, inv_fy, cx_fx, cy_fy)

        let res = Array.zeroCreate cnt

        for i in 0..cnt-1 do
            let R = Rs.[i]
            let t = ts.[i]

            res.[i] <- (R,t)

        res

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
                    descriptors             = Array.empty<byte>
                    descriptorDimension     = 61
                }
            else
                let pts : KeyPoint2d[] = Array.zeroCreate v.PointCount
                copy v.Points pts pts.Length
                let mutable typ = null
                let descriptors =
                    match v.DescriptorElementType with
                    | ElementType.Float ->
                        typ <- typeof<float32>
                        let descriptors : float32[] = Array.zeroCreate v.DescriptorEntries
                        copy (NativePtr.cast v.Descriptors) descriptors descriptors.Length
                        descriptors :> System.Array
                    | ElementType.Double ->
                        typ <- typeof<float>
                        let descriptors : float[] = Array.zeroCreate v.DescriptorEntries
                        copy (NativePtr.cast v.Descriptors) descriptors descriptors.Length
                        descriptors :> System.Array
                    | ElementType.Int8 ->
                        typ <- typeof<int8>
                        let descriptors : int8[] = Array.zeroCreate v.DescriptorEntries
                        copy (NativePtr.cast v.Descriptors) descriptors descriptors.Length
                        descriptors :> System.Array
                    | ElementType.UInt8 ->
                        typ <- typeof<uint8>
                        let descriptors : uint8[] = Array.zeroCreate v.DescriptorEntries
                        copy v.Descriptors descriptors descriptors.Length
                        descriptors :> System.Array
                    | ElementType.UInt16 ->
                        typ <- typeof<uint16>
                        let descriptors : uint16[] = Array.zeroCreate v.DescriptorEntries
                        copy (NativePtr.cast v.Descriptors) descriptors descriptors.Length
                        descriptors :> System.Array
                    | ElementType.Int16 ->
                        typ <- typeof<int16>
                        let descriptors : int16[] = Array.zeroCreate v.DescriptorEntries
                        copy (NativePtr.cast v.Descriptors) descriptors descriptors.Length
                        descriptors :> System.Array
                    | ElementType.Int32 ->
                        typ <- typeof<int>
                        let descriptors : int[] = Array.zeroCreate v.DescriptorEntries
                        copy (NativePtr.cast v.Descriptors) descriptors descriptors.Length
                        descriptors :> System.Array
                    | t ->
                        failwithf "bad descriptor type: %A" t

                let dim = descriptors.Length / pts.Length
                let points = pts |> Array.mapi (fun i pt -> KeyPoint(i, V2d pt.pt, float pt.size, float pt.angle, float pt.response, pt.octave, dim, typ, descriptors))
                {
                    points                  = points
                    descriptors             = descriptors
                    descriptorDimension     = dim
                }

        finally
            Native.cvFreeFeatures_ ptr
            gc.Free()

    let codeList = 
        [|
            0xB905C6D4BUL
            0xA72C88006UL
            0x4E7DF6048UL
            0x2D36B51FFUL
            0x923D95BA1UL
            0x397F5862DUL
            0xC87F8ADB6UL
            0xF82A3124AUL
            0xC0A2CF100UL
            0x737462571UL
            0xB6E7D4357UL
            0x223A86F58UL
            0xE4F92512DUL
            0xDC1487274UL
            0x15F4F8D1EUL
            0x7EAF2CBE4UL
            0x8FFF44491UL
            0x602966832UL
            0x88ACD92FDUL
            0xBAB38983BUL
            0xA1FC3EB88UL
            0xAC70C8568UL
            0xFF7E4F947UL
            0x30751B142UL
            0xEE212AB09UL
            0xC93D398C4UL
            0x3FBC30DC3UL
            0x9333D2420UL
            0x0C6B81054UL
            0x69BC2E46EUL
            0x98FDA260BUL
            0x06BFEB1CFUL
            0x0859FD363UL
            0xA63E7D83CUL
            0xC67AEF6E4UL
            0xC6FC50922UL
            0x4AB35065BUL
            0x9AFB1633CUL
            0x75B9F62D5UL
            0x1EBCCAB11UL
            0x843746B6DUL
            0x626D403CDUL
            0x102B6A744UL
            0x50EEAFA35UL
            0x66FC99555UL
            0xB13E05532UL
            0x0DEF2D93AUL
            0xA070F7955UL
            0x0AEA3A0E0UL
            0x7A2AF8135UL
            0x2E789DACEUL
            0x2773AC331UL
            0x20B296227UL
            0x07B8487ADUL
            0xF0BE80864UL
            0xAD79C2B8FUL
            0x6935CC585UL
            0x4034632E8UL
            0x68EFE4778UL
            0x9737A38D1UL
            0x5630DA056UL
            0x3FB16A0B0UL
            0xFA7A4B498UL
            0x13337DB58UL
            0x00FE137F3UL
            0x5E7BB9D0BUL
            0x781F73B11UL
            0x9D3816A22UL
            0xC13FF4DDBUL
            0x8FB69B125UL
            0xBE74334EDUL
            0xFCADA9430UL
            0x20E4C192FUL
            0x943B71699UL
            0xA1F587279UL
            0xD8B47C161UL
            0x203A2D081UL
            0x47FD22356UL
            0xA8662CA26UL
            0x19ABD099CUL
            0x56F0B7938UL
            0xA6B94E643UL
            0xA9EFC99C1UL
            0xBA35A035CUL
            0xDCFFB5EACUL
            0xA6AA03AB0UL
            0x22F7E2AF4UL
            0x820A88361UL
            0xFA79BF026UL
            0xEF3DD1073UL
            0x7C2CDD908UL
            0x493AC2843UL
            0x273FB2DACUL
            0x4F34AE9A2UL
            0x341DA8937UL
            0x0B778118AUL
            0xE93BE5BB4UL
            0x228428074UL
            0x99F091742UL
            0xE4A6A1159UL
            0x01B9B0171UL
            0x2CF3D3891UL
            0x08241BBA4UL
            0x2CB8AA9C4UL
            0x0E30F17F1UL
            0x0E9560829UL
            0x302D90641UL
            0x6CF5393F0UL
            0x88B818116UL
            0x103C8618EUL
            0x946ED58F0UL
            0x183FDEAC5UL
            0x625EDC032UL
            0x673B7B204UL
            0xECB00C6B5UL
            0x90B323321UL
            0x23A706842UL
            0x15FB54BE1UL
            0x92EE70A1EUL
            0x3E3F0AE72UL
            0x5BFBA74FDUL
            0xA5B24C11BUL
            0x01F4AD2C4UL
            0x28A2041ECUL
            0xCAB54FBD5UL
            0x9F71E7A6CUL
            0x453655392UL
            0xCFBB9C3C0UL
            0xBA1CA3163UL
            0xE6FDD3FDAUL
            0x847824C46UL
            0x33AB093D5UL
            0xA17F6311CUL
            0xB46B4705BUL
            0x97FBAA108UL
            0xD6BF762B2UL
            0x86B48576EUL
            0xBCBEECDB9UL
            0x8E5F70B77UL
            0xF475C3767UL
            0xC1775F822UL
            0xAFF13985DUL
            0xD57AC0C8CUL
            0xA46EF1736UL
            0xB1AD5B011UL
            0xB07902CB1UL
            0x8477497D4UL
            0x36340C06BUL
            0x24E3FF962UL
            0xCB77D77E1UL
            0x5EF7E1A42UL
            0x4A723315EUL
            0x0A6EC256EUL
            0x2F7827999UL
            0x4279690BEUL
            0xD62F5D403UL
            0xE27CF9A49UL
            0x03B856884UL
            0x21B27B4F9UL
            0x9435FF700UL
            0x31FF8ECDEUL
            0x95B9CC166UL
            0xFC775080BUL
            0x032D00B38UL
            0xDC3F0E0A9UL
            0x0EE59321FUL
            0x6EBC61F1CUL
            0xA20FF626CUL
            0x48269D873UL
            0x15BA2BA5EUL
            0x36F700728UL
            0x1BFD5A97AUL
            0x94ED3974DUL
            0xEBBE236C0UL
            0xD63E9771DUL
            0xEA6F17970UL
            0x58F74B52BUL
            0x4CFE38811UL
            0x8C3E7BDE2UL
            0x26DFB5B5CUL
            0x2EA14DA76UL
            0x89B3E67CCUL
            0xB4BBF9D54UL
            0xD4FE65F51UL
            0x7A3F14567UL
            0xBDB39204EUL
            0x4835BA675UL
            0x64A258A48UL
            0x821D59108UL
            0x077690F47UL
            0xCEA77530DUL
            0x617EE66BFUL
            0x18AFCD222UL
            0x56A5CD17DUL
            0x3F3AE5016UL
            0x1575D4035UL
            0x387A81BE9UL
            0x10761C2B8UL
            0x70DAF2858UL
            0x81B151A67UL
            0xE5F7161C5UL
            0xCE695166CUL
            0xA1BCA3F17UL
            0xABB7ED474UL
            0xC8F0E24D1UL
            0x4CFA9E96FUL
            0x713FF91A2UL
            0x393010474UL
            0xBEFA64260UL
            0x08BB28EBCUL
            0x8F7634968UL
            0x92F4FE67CUL
            0xC6B5B820AUL
            0xA82C0A858UL
            0x42E318326UL
            0xAC3774E50UL
            0x063B9F0F8UL
            0xE63D9AA95UL
            0xB0B8894CFUL
            0x3FF2B0384UL
            0x0DAF73807UL
            0x04AC5C472UL
            0x92720F255UL
            0xC233F2387UL
            0x176F17104UL
            0xA52EE6121UL
            0x1AAE31979UL
            0x3BEDF9A34UL
            0xC86256229UL
            0x7433B4941UL
            0x8C686B200UL
            0x901E4AA0BUL
            0xB4A1C6805UL
            0x9C6FEA6F1UL
            0xEC7C36533UL
            0x5E3D6C8D3UL
            0xCA7C15A3FUL
            0x0C3F22559UL
            0x3CBD837BDUL
            0x3EF9D0407UL
            0x0DFACF1D2UL
            0xEC6FA681DUL
            0x96B113BCCUL
            0xB27EF4508UL
            0xCAFFAD313UL
            0xD6FF9A47BUL
            0x2BB0E0B62UL
            0x82E9DCB4CUL
            0x2038FAE0EUL
            0x057DB1AB2UL
        |]

    let createArucoMarker (minPixels : int) (value : int) =
        let cellSize = ceil (float minPixels / 10.0) |> int
        let result = PixImage<byte>(Col.Format.Gray, V2i(10,10) * cellSize)
        let mutable res = result.GetChannel(0L)
        res.Set(255uy) |> ignore
        res.SubMatrix(V2i(cellSize, cellSize), 8*V2i(cellSize, cellSize)).Set(0uy) |> ignore

        let code = codeList.[value]
        let inline hasBit (i : int) = ((code >>> i) &&& 1UL) <> 0UL
        for y in 0 .. 5 do
            for x in 0 .. 5 do
                let v = hasBit (y * 6 + x)

                let cell = V2i(x+2, y+2)
                let dst = res.SubMatrix(cell * cellSize, V2i cellSize)
                dst.Set(if v then 255uy else 0uy) |> ignore

        result



          
    let detectArucoMarkers (img : PixImage<byte>) =
        let img = img.ToCanonicalDenseLayout() |> unbox<PixImage<byte>>

        let gc = GCHandle.Alloc(img.Volume.Data, GCHandleType.Pinned)
        try

            let infos = Array.zeroCreate 256
            let mutable ic = infos.Length

            use pInfos = fixed infos
            let res = 
                Native.cvDetectArucoMarkers(
                    NativePtr.ofNativeInt (gc.AddrOfPinnedObject()), img.Size.X, img.Size.Y, img.ChannelCount,
                    &ic, pInfos
                )
            if res then
                Array.take ic infos
            else
                [||]


        finally
            gc.Free()
        

    let detectQRCode (img : PixImage<byte>) =
        let img = img.ToCanonicalDenseLayout() |> unbox<PixImage<byte>>

        let gc = GCHandle.Alloc(img.Volume.Data, GCHandleType.Pinned)
        try
            let positions = NativePtr.stackalloc<V2i> 32
            let mutable cnt = 32
            let worked = Native.cvDetectQRCode(NativePtr.ofNativeInt (gc.AddrOfPinnedObject()), img.Size.X, img.Size.Y, img.ChannelCount, positions, &cnt)
            if worked then
                Array.init cnt (fun i ->
                    NativePtr.get positions i
                )
            else
                [||]
        finally
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
            
    [<Struct>]
    type RefineResult =
        {
            t : V3d
            r : V3d
        }

    /// virtual visual servoing https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga17491c0282e4af874f6206a9166774a5
    let refinePnPVVS (imgPoints : V2d[]) (worldPoints : V3d[]) (intern : M33d) (distortionCoeffs : float[]) (t : V3d) (r : V3d) =
        let mutable tRes = t
        let mutable rRes = r
        Native.cvRefinePnPVVS(imgPoints, worldPoints, worldPoints.Length, intern, distortionCoeffs, &tRes, &rRes)
        let r = rRes
        let trn = tRes
        {
            t = trn
            r = r
        }

    let refinePnPLM (imgPoints : V2d[]) (worldPoints : V3d[]) (intern : M33d) (distortionCoeffs : float[]) (t : V3d) (r : V3d) =
        let mutable tRes = Unchecked.defaultof<_>
        let mutable rRes = Unchecked.defaultof<_>
            
        Native.cvRefinePnPLM(imgPoints, worldPoints, worldPoints.Length, intern, distortionCoeffs, &tRes, &rRes)
        let r = rRes
        let trn = tRes
        {
            t = trn
            r = r
        }
        
    type RefineKind =
    | LM
    | VVS
    type SolverKind =
    | EPNP
    | P3P
    | AP3P
    | Iterative
    | SQPNP
    type RansacParams =
        {
            reprojectionError : float32
            iterationsCount : int
            confidence : float
        }
        with 
            static member MaxReprojError (e : float) = 
                {
                    reprojectionError = float32 e
                    iterationsCount = 100
                    confidence = 0.99
                }
    
    let solvePnPInternal (solver : SolverKind) (refinement : Option<RefineKind>) (ransac : Option<RansacParams>) (imgPoints : V2d[]) (worldPoints : V3d[]) (intern : M33d) (distortionCoeffs : float[]) =
        if imgPoints.Length <> worldPoints.Length then
            None
        else
            let kind =
                match solver with
                | Iterative -> 0
                | EPNP -> 1
                | P3P -> 2
                | AP3P -> 5
                | SQPNP -> 6
        
            let mutable tRes = Unchecked.defaultof<_>
            let mutable rRes = Unchecked.defaultof<_>
            let mutable inlierCount = 0
            
            let mutable icnt = 0
            let mutable inliers = null

            let converged = 
                match ransac with 
                | Some {reprojectionError = reproj; iterationsCount = iter; confidence = confidence} -> 
                    inliers <- Array.zeroCreate imgPoints.Length
                    Native.cvSolvePnPRansac(imgPoints, worldPoints, worldPoints.Length, intern, distortionCoeffs, kind, iter, reproj, confidence, &tRes, &rRes, &icnt, inliers)
                | None -> Native.cvSolvePnP(imgPoints, worldPoints, worldPoints.Length, intern, distortionCoeffs, kind, &tRes, &rRes)
            if converged then
                let r = rRes
                let t = tRes

                
                let inlierImgPoints   = if isNull inliers then imgPoints else Array.init icnt (fun i -> let i = inliers.[i] in imgPoints.[i])
                let inlierWorldPoints = if isNull inliers then worldPoints else Array.init icnt (fun i -> let i = inliers.[i] in worldPoints.[i])
                    
                let t,r = 
                    match refinement with 
                    | Some LM ->  let {t=t;r=r} = refinePnPLM  inlierImgPoints inlierWorldPoints intern distortionCoeffs t r in t,r
                    | Some VVS -> let {t=t;r=r} = refinePnPVVS inlierImgPoints inlierWorldPoints intern distortionCoeffs t r in t,r
                    | None -> t,r

                let ang = r.Length
                let axs = r.Normalized
                let rotOrig = Rot3d.Rotation(axs,ang)
                
                let rot, trn =
                    match solver with
                    | Iterative ->
                        let rot = Rot3d.FromAngleAxis(V3d.OOI * Constant.Pi) * rotOrig
                        let trn = rot.Transform(rotOrig.InvTransform(t))
                        rot,trn
                    | EPNP ->
                        let rot = rotOrig
                        let trn = t
                        rot,trn
                    | _ ->
                        let rot = rotOrig
                        let trn = t
                        rot,trn
                        
                let e = Euclidean3d(rot,trn)
                
                Some (e,inliers)
            else
                None
    
    let solvePnP (solver : SolverKind) (imgPoints : V2d[]) (worldPoints : V3d[]) (intern : M33d) (distortionCoeffs : float[]) =
        solvePnPInternal solver None None imgPoints worldPoints intern distortionCoeffs
        |> Option.map (fun (e,inl) -> e)
    
    let solvePnPWithRefine (solver : SolverKind) (refine : RefineKind) (imgPoints : V2d[]) (worldPoints : V3d[]) (intern : M33d) (distortionCoeffs : float[]) =
        solvePnPInternal solver (Some refine) None imgPoints worldPoints intern distortionCoeffs
        |> Option.map (fun (e,inl) -> e)

    let solvePnPRansac (solver : SolverKind) (cfg : RansacParams) (imgPoints : V2d[]) (worldPoints : V3d[]) (intern : M33d) (distortionCoeffs : float[]) =
        solvePnPInternal solver None (Some cfg) imgPoints worldPoints intern distortionCoeffs
    
    let solvePnPRansacWithRefine (solver : SolverKind) (cfg : RansacParams) (refine : RefineKind) (imgPoints : V2d[]) (worldPoints : V3d[]) (intern : M33d) (distortionCoeffs : float[]) =
        solvePnPInternal solver (Some refine)  (Some cfg) imgPoints worldPoints intern distortionCoeffs