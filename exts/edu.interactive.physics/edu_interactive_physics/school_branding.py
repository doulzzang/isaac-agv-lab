"""Visual-only Dongshin school signs for the existing warehouse room.

The official 258 x 50 RGBA logo is used unchanged.  This module has no Kit,
physics, or UI dependencies and can also author an offline USD stage.
"""

import hashlib
from pathlib import Path

from pxr import Gf, Sdf, UsdGeom, UsdShade


SCHOOL_NAME = "광주동신여자고등학교"
SCHOOL_URL = "https://kdsg.jge.hs.kr/main/main.php"
LOGO_URL = "https://kdsg.jge.hs.kr/data_files/skin/skin_res_1/images/common/logo_on.png"
LOGO_SHA256 = "5f3201a4980c276e89fe00f0872eea5467f91c362d1aa9728012979d25d834a9"
LOGO_WIDTH_PX = 258
LOGO_HEIGHT_PX = 50
LOGO_WIDTH_M = 4.0
LOGO_HEIGHT_M = LOGO_WIDTH_M * LOGO_HEIGHT_PX / LOGO_WIDTH_PX
SURFACE_SEPARATION_M = 0.002


def _prim_path(value, name):
    path = Sdf.Path(str(value).rstrip("/"))
    if not path.IsAbsolutePath() or not path.IsPrimPath():
        raise ValueError(f"{name} must be an absolute USD prim path: {value}")
    return str(path)


def _solid_material(stage, path, color):
    material = UsdShade.Material.Define(stage, path)
    surface = UsdShade.Shader.Define(stage, f"{path}/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    surface.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
    surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    return material


def _logo_material(stage, path, texture_path):
    material = _solid_material(stage, path, (1.0, 1.0, 1.0))
    surface = UsdShade.Shader(stage.GetPrimAtPath(f"{path}/Surface"))
    reader = UsdShade.Shader.Define(stage, f"{path}/PrimvarReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    texture = UsdShade.Shader.Define(stage, f"{path}/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture_path.as_posix()))
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    texture.CreateOutput("a", Sdf.ValueTypeNames.Float)
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
    # Use the official PNG alpha channel to cut out its transparent background.
    surface.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(texture.ConnectableAPI(), "a")
    # A printed sign needs alpha cutout rather than a transmissive surface.
    # RTX otherwise darkens transparent texels in front of the white backing.
    surface.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.1)
    surface.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.0)
    return material


def _quad(stage, path, center, right, up, width, height, material, color):
    horizontal = right * (width / 2.0)
    vertical = up * (height / 2.0)
    normal = Gf.Cross(right, up).GetNormalized()
    points = [center - horizontal - vertical, center + horizontal - vertical,
              center + horizontal + vertical, center - horizontal + vertical]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
    mesh.CreateDoubleSidedAttr(False)
    mesh.CreateNormalsAttr([normal] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateExtentAttr([
        Gf.Vec3f(*(min(p[axis] for p in points) for axis in range(3))),
        Gf.Vec3f(*(max(p[axis] for p in points) for axis in range(3))),
    ])
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    primvars = UsdGeom.PrimvarsAPI(mesh)
    primvars.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set([
        Gf.Vec2f(0.0, 0.0), Gf.Vec2f(1.0, 0.0),
        Gf.Vec2f(1.0, 1.0), Gf.Vec2f(0.0, 1.0),
    ])
    # RTX's geometry property is also authored by the installed Isaac Sim SDK.
    primvars.CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool, UsdGeom.Tokens.constant).Set(True)
    mesh.GetPrim().CreateAttribute("lab:branding", Sdf.ValueTypeNames.String).Set("dongshin")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    # No CollisionAPI, RigidBodyAPI, or contact geometry is created.
    return mesh


def _sign(stage, path, center, right, up, materials, is_floor):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    prim.CreateAttribute("lab:branding", Sdf.ValueTypeNames.String).Set("dongshin")
    prim.CreateAttribute("lab:objectType", Sdf.ValueTypeNames.String).Set("floor_sign" if is_floor else "wall_sign")
    prim.CreateAttribute("lab:school", Sdf.ValueTypeNames.String).Set(SCHOOL_NAME)
    prim.CreateAttribute("lab:source", Sdf.ValueTypeNames.String).Set("assets/branding/dongshin_school_logo.png")
    prim.CreateAttribute("lab:sourceUrl", Sdf.ValueTypeNames.String).Set(LOGO_URL)
    prim.CreateAttribute("lab:visualOnly", Sdf.ValueTypeNames.Bool).Set(True)
    center, right, up = Gf.Vec3f(*center), Gf.Vec3f(*right), Gf.Vec3f(*up)
    normal = Gf.Cross(right, up).GetNormalized()
    board_width, board_height = LOGO_WIDTH_M + 0.24, LOGO_HEIGHT_M + 0.20
    _quad(stage, f"{path}/Frame", center, right, up,
          board_width + 0.06, board_height + 0.06, materials["frame"], (0.20, 0.24, 0.23))
    _quad(stage, f"{path}/WhiteBacking", center + normal * SURFACE_SEPARATION_M, right, up,
          board_width, board_height, materials["white"], (1.0, 1.0, 1.0))
    _quad(stage, f"{path}/LogoMesh", center + normal * (2 * SURFACE_SEPARATION_M), right, up,
          LOGO_WIDTH_M, LOGO_HEIGHT_M, materials["logo"], (1.0, 1.0, 1.0))
    return path


def create_dongshin_branding(stage, lab_root="/World/InteractivePhysicsLab", object_root=None):
    """Create or update school signs only where the warehouse surfaces exist.

    Coordinates follow the existing warehouse/GIST room.  Right/up vectors
    make each wall's front face point into the room; the floor faces +Z.
    Repeated calls update the same Dongshin paths without touching GIST signs.
    Returns JSON-serializable paths and counts for the caller's creation proof.
    """
    if stage is None:
        raise ValueError("A USD stage is required")
    lab_root = _prim_path(lab_root, "lab_root")
    object_root = _prim_path(object_root or f"{lab_root}/Objects", "object_root")
    texture_path = Path(__file__).resolve().parent.parent / "assets" / "branding" / "dongshin_school_logo.png"
    if not texture_path.is_file():
        raise FileNotFoundError(f"Official Dongshin logo not found: {texture_path}")
    digest = hashlib.sha256(texture_path.read_bytes()).hexdigest()
    if digest != LOGO_SHA256:
        raise ValueError("The Dongshin logo differs from the unchanged official source PNG")

    branding_root = f"{lab_root}/Branding"
    surfaces = (
        (f"{object_root}/wall_right", "DongshinWallSign", (5.0, -5.895, 1.7), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0), False),
        (f"{object_root}/wall_left", "DongshinWallSignLeft", (5.0, 5.895, 1.7), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), False),
        (f"{object_root}/wall_back", "DongshinWallSignBack", (10.895, 0.0, 1.7), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0), False),
        (f"{lab_root}/Ground", "DongshinFloorSign", (-0.5, 0.0, 0.005), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0), True),
    )
    present, skipped = [], []
    for surface in surfaces:
        if stage.GetPrimAtPath(surface[0]).IsValid():
            present.append(surface)
        else:
            skipped.append(surface[0])
            # Remove only a previously authored Dongshin sign for this surface.
            old = stage.GetPrimAtPath(f"{branding_root}/{surface[1]}")
            if old.IsValid() and old.GetAttribute("lab:branding").Get() == "dongshin":
                stage.RemovePrim(old.GetPath())

    paths = []
    if present:
        UsdGeom.Xform.Define(stage, branding_root)
        material_root = f"{branding_root}/DongshinMaterials"
        UsdGeom.Scope.Define(stage, material_root)
        materials = {
            "frame": _solid_material(stage, f"{material_root}/Frame", (0.20, 0.24, 0.23)),
            "white": _solid_material(stage, f"{material_root}/White", (1.0, 1.0, 1.0)),
            "logo": _logo_material(stage, f"{material_root}/Logo", texture_path),
        }
        for _, name, center, right, up, is_floor in present:
            paths.append(_sign(stage, f"{branding_root}/{name}", center, right, up, materials, is_floor))
    return {
        "branding": "dongshin",
        "school": SCHOOL_NAME,
        "school_url": SCHOOL_URL,
        "logo_source_url": LOGO_URL,
        "logo_asset": texture_path.as_posix(),
        "logo_sha256": digest,
        "logo_size_px": [LOGO_WIDTH_PX, LOGO_HEIGHT_PX],
        "logo_size_m": [LOGO_WIDTH_M, LOGO_HEIGHT_M],
        "visual_only": True,
        "wall_count": sum(not item[5] for item in present),
        "floor_created": any(item[5] for item in present),
        "created_count": len(paths),
        "paths": paths,
        "skipped_surfaces": skipped,
    }
