namespace rec MiniCVTest

open System
open MiniCV
open Aardvark.Base


open Aardvark.Base

[<StructuredFormatDisplay("{AsString}")>]
type Projection =
    {
        focalLength     : float
        aspect          : float
        principalPoint  : V2d
    }

    member private x.AsString =
        if x.principalPoint = V2d.Zero then
            sprintf "Projection { focal = %A; aspect = %A; pp = %A }"
                x.focalLength
                x.aspect
                x.principalPoint

        else
            sprintf "Projection { focal = %A; aspect = %A }"
                x.focalLength
                x.aspect

    override x.ToString() = x.AsString

    static member inline (+) (l : Projection, r : Projection) = Projection.compose l r
    static member inline (-) (l : Projection, r : Projection) = Projection.compose l (Projection.inverse r)

    member x.Trafo = Projection.toTrafo x
    member x.Inverse = Projection.inverse x

    member x.Project pt = Projection.project pt x
    member x.ProjectUnsafe pt = Projection.projectUnsafe pt x
    member x.Unproject pt = Projection.unproject pt x

module Projection =

    let identity =
        {
            aspect = 1.0
            focalLength = 1.0
            principalPoint = V2d.Zero
        }

    let approxEqual (eps : float) (l : Projection) (r : Projection) =
        Fun.ApproximateEquals(l.focalLength, r.focalLength, eps) &&
        Fun.ApproximateEquals(l.aspect, r.aspect, eps) &&
        Fun.ApproximateEquals(l.principalPoint, r.principalPoint, eps)

    let project (p : V3d) (proj : Projection) =
        let z = p.Z
        let p = (proj.focalLength * p.XY * V2d(1.0, proj.aspect) + proj.principalPoint * z)
        if p.X >= -z && p.X <= z && p.Y >= -z && p.Y <= z && z > 0.0 then
            (p / z) |> Some
        else
            None

    let projectUnsafe (p : V3d) (proj : Projection) =
        (proj.focalLength * p.XY * V2d(1.0, proj.aspect)) / p.Z + proj.principalPoint

    let unproject (p : V2d) (proj : Projection) =
        // p.Z = 1
        // (w - proj.principalPoint) / (proj.focalLength * V2d(1.0, proj.aspect)) = p.XY
        V3d((p - proj.principalPoint) / (proj.focalLength * V2d(1.0, proj.aspect)), 1.0)

    let inverse (proj : Projection) =
        //M33d(
        //    1.0/proj.focalLength,   0.0,                                    -proj.principalPoint.X / proj.focalLength,
        //    0.0,                    1.0 / (proj.aspect * proj.focalLength), -proj.principalPoint.Y / (proj.focalLength * proj.aspect),
        //    0.0,                    0.0,                                     1.0
        //)
        let px = -proj.principalPoint.X / proj.focalLength
        let py = -proj.principalPoint.Y / (proj.focalLength * proj.aspect)

        { 
            aspect = 1.0 / proj.aspect
            focalLength = 1.0 / proj.focalLength
            principalPoint = V2d(px,py)
        }
        
    let compose (l : Projection) (r : Projection) =
        let f = l.focalLength
        let g = r.focalLength
        let p = l.principalPoint.X
        let q = l.principalPoint.Y
        let u = r.principalPoint.X
        let v = r.principalPoint.Y
        let a = l.aspect
        let b = r.aspect

        let pp =
            V2d (
                (p + f * u),
                (q + a*f*v)
            )

        {
            aspect = a * b
            focalLength = f * g
            principalPoint = pp
        }

    let toTrafo (proj : Projection) =
        Trafo2d(
            M33d(
                proj.focalLength,           0.0,                                proj.principalPoint.X,
                0.0,                        proj.focalLength * proj.aspect,     proj.principalPoint.Y,
                0.0,                        0.0,                                1.0
            ),
            M33d(
                1.0/proj.focalLength,   0.0,                                    -proj.principalPoint.X / proj.focalLength,
                0.0,                    1.0 / (proj.aspect * proj.focalLength), -proj.principalPoint.Y / (proj.focalLength * proj.aspect),
                0.0,                    0.0,                                     1.0
            )
        )


[<StructuredFormatDisplay("{AsString}")>]
type CameraView =
    {
        trafo : Euclidean3d
    }

    member private x.AsString =
        sprintf "CameraView { location = %A; forward = %A; right = %A; up = %A }"
            x.Location
            x.Forward
            x.Right
            x.Up

    override x.ToString() = x.AsString

    static member inline (+) (c : CameraView, m : CameraMotion) = CameraView.move m c
    static member inline (-) (c : CameraView, m : CameraMotion) = CameraView.move (CameraMotion.inverse m) c
    static member inline (-) (l : CameraView, r : CameraView) = CameraView.motion r l

    member x.Location = CameraView.location x
    member x.Forward = CameraView.forward x
    member x.Right = CameraView.right x
    member x.Up = CameraView.up x
    member x.ViewTrafo = CameraView.viewTrafo x

module CameraView =

    let approxEqual (eps : float) (l : CameraView) (r : CameraView) =
        (Fun.ApproximateEquals(l.trafo.Rot, r.trafo.Rot, eps) || Fun.ApproximateEquals(-l.trafo.Rot, r.trafo.Rot, eps)) &&
        Fun.ApproximateEquals(l.trafo.Trans, r.trafo.Trans, eps)
        //Euclidean3d.ApproxEqual(l.trafo, r.trafo, eps, eps)


    // forward = -z
    // f = 1
    // a = 1
    let identity =
        {
            trafo = Euclidean3d.Identity
        }

    let forward (c : CameraView) = c.trafo.InvTransformDir(-V3d.OOI)
    let right (c : CameraView) = c.trafo.InvTransformDir(V3d.IOO)
    let up (c : CameraView) = c.trafo.InvTransformDir(V3d.OIO)
    let location (c : CameraView) = c.trafo.Rot.InvTransform(-c.trafo.Trans)


    let lookAt (eye : V3d) (center : V3d) (sky : V3d) =
        let fv = Vec.normalize (center - eye)
        let rv = Vec.cross fv sky |> Vec.normalize
        let uv = Vec.cross rv fv |> Vec.normalize
        
        //  r => IOO
        //  u => OIO
        // -f => OOI
        // -r.TransformPos(eye) = r.Trans;

        let r = Rot3d.FromFrame(rv, uv, -fv).Inverse
        let t = -r.Transform(eye)
        let e = Euclidean3d(r, t)
        { trafo = e }

    let viewTrafo (c : CameraView) =
        Trafo3d(
            Euclidean3d.op_Explicit c.trafo,
            Euclidean3d.op_Explicit c.trafo.Inverse
        )

    let motion (l : CameraView) (r : CameraView) =
        { 
            trafo = r.trafo * l.trafo.Inverse
            isNormalized = false
        }
    
    let move (motion : CameraMotion) (cam : CameraView) =
        { cam with trafo = motion.trafo * cam.trafo }

[<StructuredFormatDisplay("{AsString}")>]
type Camera = { view : CameraView; proj : Projection} with
    
    member private x.AsString =
        sprintf "Camera { view = %A; proj = %A }"
            x.view
            x.proj

    override x.ToString() = x.AsString
    
    static member inline (+) (c : Camera, m : CameraMotion) = Camera.move m c
    static member inline (-) (c : Camera, m : CameraMotion) = Camera.move (CameraMotion.inverse m) c
    static member inline (-) (l : Camera, r : Camera) = Camera.motion r l

    member x.Location = Camera.location x
    member x.Forward = Camera.forward x
    member x.Right = Camera.right x
    member x.Up = Camera.up x
    member x.ViewTrafo = Camera.viewTrafo x
    member x.ProjTrafo(near, far) = Camera.projTrafo near far
    member x.ViewProjTrafo(near, far) = Camera.viewProjTrafo near far

    member x.Project pt = Camera.project pt x
    member x.ProjectUnsafe pt = Camera.projectUnsafe pt x
    member x.Unproject pt = Camera.unproject pt x

module Camera =

    let location (c : Camera) = CameraView.location c.view
    let forward (c : Camera) = CameraView.forward c.view
    let right (c : Camera) = CameraView.right c.view
    let up (c : Camera) = CameraView.up c.view


    let projTrafo (near : float) (far : float) (cam : Camera) =
        let proj = cam.proj
        let tnf = 2.0 * far * near
        let a = (far + near) / (far - near)
        let b = tnf / (near - far)


        // (near * a + b) / near = -1
        // (far * a + b) / far = 1

        // a + b / near = -1
        // a + b / far = 1
        
        // near * a + b = -near     * far
        // far * a + b = far        * near

        // near * a + b = -near   
        // far * a + b = far      
        // a = (near + far) / (far - near)

        // far * b - near * b = -2*near*far
        // b = 2*near*far / (near - far)

        Trafo3d(
            M44d(
                proj.focalLength,           0.0,                               -proj.principalPoint.X,  0.0,
                0.0,                        proj.focalLength * proj.aspect,    -proj.principalPoint.Y,  0.0,
                0.0,                        0.0,                               -a,                      b,
                0.0,                        0.0,                               -1.0,                    0.0
            ),
            M44d(
                1.0/proj.focalLength,   0.0,                                    0.0,       -proj.principalPoint.X / proj.focalLength,
                0.0,                    1.0 / (proj.aspect * proj.focalLength), 0.0,       -proj.principalPoint.Y / (proj.focalLength * proj.aspect),
                0.0,                    0.0,                                    0.0,       -1.0,
                0.0,                    0.0,                                    1.0/b,     -a/b

            )
        )
        
    let viewTrafo (c : Camera) =
        CameraView.viewTrafo c.view

    let viewProjTrafo (near : float) (far : float) (c : Camera) =
        CameraView.viewTrafo c.view * projTrafo near far c
        
    let projectUnsafe (pt : V3d) (c : Camera) =
        let viewSpace = c.view.trafo.TransformPos pt
        let viewSpace = V3d(viewSpace.XY, -viewSpace.Z)
        Projection.projectUnsafe viewSpace c.proj
        
    let project (pt : V3d) (c : Camera) =
        let viewSpace = c.view.trafo.TransformPos pt
        let viewSpace = V3d(viewSpace.XY, -viewSpace.Z)
        Projection.project viewSpace c.proj
        
    let unproject (pt : V2d) (c : Camera) =
        let p = Projection.unproject pt c.proj
        let p = V3d(p.XY, -p.Z)
        let pp = c.view.trafo.InvTransformDir p
        let dir = Vec.normalize pp
        let c = CameraView.location c.view
        Ray3d(c, dir)

    let move (motion : CameraMotion) (c : Camera) =
        { c with view = CameraView.move motion c.view }

    let motion (l : Camera) (r : Camera) =
        CameraView.motion l.view r.view

    let approxEqual (eps : float) (l : Camera) (r : Camera) =
        (CameraView.approxEqual eps l.view r.view) && (Projection.approxEqual eps l.proj r.proj)

[<StructuredFormatDisplay("{AsString}")>]
type CameraMotion =
    {
        trafo : Euclidean3d
        isNormalized : bool
    }

    member private x.AsString =
        let r = x.trafo.Rot.GetEulerAngles() * Constant.DegreesPerRadian
        let t = x.trafo.Trans
        sprintf "CameraMotion { rot = [%.3f°, %.3f°, %.3f°]; trans = %A }" r.X r.Y r.Z t

    override x.ToString() = x.AsString


    member inline x.Normalized = CameraMotion.normalize x
    member inline x.Inverse = CameraMotion.inverse x
    
    static member inline (~-) (m : CameraMotion) = CameraMotion.inverse m
    static member inline (*) (m : CameraMotion, f : float) = CameraMotion.scale f m
    static member inline (*) (f : float, m : CameraMotion) = CameraMotion.scale f m
    static member inline (+) (l : CameraMotion, r : CameraMotion) = CameraMotion.compose l r
    static member Zero = CameraMotion.zero

module CameraMotion =
    
    let zero = { trafo = Euclidean3d.Identity; isNormalized = false }

    let approxEqual (eps : float) (l : CameraMotion) (r : CameraMotion) =
        (Fun.ApproximateEquals(l.trafo.Rot, r.trafo.Rot, eps) || Fun.ApproximateEquals(-l.trafo.Rot, r.trafo.Rot, eps)) &&
        Fun.ApproximateEquals(l.trafo.Trans, r.trafo.Trans, eps)

    let normalize (m : CameraMotion) =
        if m.isNormalized then
            m
        elif Fun.ApproximateEquals(m.trafo.Trans, V3d.Zero, 1E-5) then
            { m with isNormalized = true }
        else
            { 
                trafo = Euclidean3d(m.trafo.Rot, Vec.normalize m.trafo.Trans)
                isNormalized = true
            }
            
    let scale (factor : float) (m : CameraMotion) =
        { 
            trafo = Euclidean3d(m.trafo.Rot, factor * m.trafo.Trans)
            isNormalized = false
        }

    let compose (l : CameraMotion) (r : CameraMotion) =
        {
            trafo = r.trafo * l.trafo
            isNormalized = false
        }

    let inverse (m : CameraMotion) =
        { m with trafo = m.trafo.Inverse }

    let ofRotationAndTrans (m : M33d) (trans : V3d) =
        let r = Rot3d.FromM33d m
        { trafo = Euclidean3d(r, r.Transform(trans)); isNormalized = false }