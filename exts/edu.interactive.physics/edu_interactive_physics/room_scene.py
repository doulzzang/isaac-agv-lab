"""Room presets and local USD environments for the lab scene builder."""
import math
import struct
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

LABELS={'warehouse':'기존 창고','empty':'빈 실습실','classroom':'교실','usd':'USD 공간'}
LAB_ITEMS={'box_01','box_02','box_04'}


def is_ai_graduate_room(config):
    return bool(config and config.get('kind') == 'usd' and Path(str(config.get('usd_path',''))).name == 'ForDemo_AI_Graduate_Building.usd')


def validate_room(data):
    if not isinstance(data,dict):raise ValueError('공간 설정을 확인하세요.')
    kind=data.get('kind','warehouse')
    if kind not in LABELS:raise ValueError('공간 종류를 선택하세요.')
    include=data.get('include_lab',True)
    if not isinstance(include,bool):raise ValueError('실습 객체 포함 여부를 확인하세요.')
    config=dict(kind=kind,include_lab=include,usd_path='',scale=1.0,offset=[0.,0.,0.])
    if kind=='usd':
        path=Path(str(data.get('usd_path',''))).expanduser().resolve()
        if not path.is_file() or path.suffix.lower() not in ('.usd','.usda','.usdc','.usdz'):
            raise ValueError('Isaac Sim PC의 USD/USDA/USDC/USDZ 공간 파일 경로를 입력하세요.')
        scale=float(data.get('scale',1))
        offset=[float(v) for v in data.get('offset',[0,0,0])]
        if not math.isfinite(scale) or not .001<=scale<=1000 or len(offset)!=3 or any(not math.isfinite(v) or abs(v)>10000 for v in offset):
            raise ValueError('크기 배율은 0.001~1000, 위치는 유한한 X/Y/Z 값이어야 합니다.')
        check=Usd.Stage.Open(str(path))
        if not check or not list(check.GetPseudoRoot().GetChildren()):raise ValueError('비어 있는 USD 공간입니다.')
        # Validate before the current room is removed. This loader treats rooms as static environments.
        if any(p.IsA(UsdPhysics.Joint) for p in check.Traverse()):
            raise ValueError('관절이 포함된 전체 시뮬레이션 대신 방/건물 환경만 내보낸 USD를 사용하세요.')
        meters=float(UsdGeom.GetStageMetersPerUnit(check))
        if not math.isfinite(meters) or meters<=0:raise ValueError('USD 길이 단위가 올바르지 않습니다.')
        if UsdGeom.GetStageUpAxis(check) not in (UsdGeom.Tokens.x,UsdGeom.Tokens.y,UsdGeom.Tokens.z):raise ValueError('지원하지 않는 공간 축입니다.')
        config.update(usd_path=str(path),scale=scale,offset=offset)
        if is_ai_graduate_room(config):config['include_lab']=False
    return config


def include_csv_object(row,config):
    if not config:return True
    kind=config['kind'];name=row.get('object_id','')
    if is_ai_graduate_room(config):return False
    if kind=='warehouse':return config['include_lab'] or name not in LAB_ITEMS
    return (kind!='usd' and row.get('object_type')=='wall') or (config['include_lab'] and name in LAB_ITEMS)


def room_focus_prim(stage, room):
    """Use the graduate building itself, excluding distant campus objects."""
    if not room or not room.IsValid():return None
    preferred=stage.GetPrimAtPath(str(room.GetPath())+'/Content/World/A_AI_Grad_Building2')
    return preferred if preferred.IsValid() else room


def building_camera_eye(bounds, aspect=16/9):
    center=bounds.GetMidpoint()
    backward=Gf.Vec3d(-.85,-1.,.62).GetNormalized()
    right=Gf.Cross(-backward,Gf.Vec3d(0,0,1)).GetNormalized()
    up=Gf.Cross(backward,right).GetNormalized()
    tan_h=20.955/(2*26.0);tan_v=15.2908/(2*26.0)
    tan_h=min(tan_h,tan_v*aspect);tan_v=min(tan_v,tan_h/aspect)
    lo,hi=bounds.GetMin(),bounds.GetMax();distance=2.2
    for x in (lo[0],hi[0]):
        for y in (lo[1],hi[1]):
            for z in (lo[2],hi[2]):
                offset=Gf.Vec3d(x,y,z)-center
                distance=max(distance,Gf.Dot(offset,backward)+1.12*max(abs(Gf.Dot(offset,right))/tan_h,abs(Gf.Dot(offset,up))/tan_v))
    return center+backward*distance


async def build_environment(ext,config,app):
    if not config or config['kind'] in ('warehouse','empty'):return
    stage=ext._stage;root='/World/InteractivePhysicsLab'
    if config['kind']=='classroom':
        for x in (3.,6.,9.):
            for y in (-3.,3.):
                name=f'desk_{int(x)}_{"left" if y>0 else "right"}'
                # Table top plus four legs are independent editable fixed objects.
                for part,position,scale in [('top',(x,y,.75),(.75,.45,.06)),('leg1',(x-.6,y-.3,.36),(.05,.05,.36)),('leg2',(x+.6,y-.3,.36),(.05,.05,.36)),('leg3',(x-.6,y+.3,.36),(.05,.05,.36)),('leg4',(x+.6,y+.3,.36),(.05,.05,.36))]:
                    ext._create_model_object(root+'/Objects/'+name+'_'+part,position,scale,(.55,.35,.16),'fixed',15)
        ext._create_model_object(root+'/Objects/whiteboard',(10.75,0,1.7),(.04,2.,.8),(.93,.95,.93),'fixed',20)
        return
    ext._set_status('공간 교체 4/8 · USD 건물 파일 읽는 중…')
    await app.next_update_async()
    source=Usd.Stage.Open(config['usd_path'])
    room=UsdGeom.Xform.Define(stage,root+'/Room')
    content=UsdGeom.Xform.Define(stage,root+'/Room/Content')
    # Root references also support files without defaultPrim. Keep Materials/Looks siblings.
    roots=list(source.GetPseudoRoot().GetChildren())
    ext._set_status('공간 교체 5/8 · 전체 처리 대상 수 확인 중…')
    await app.next_update_async()
    total=0
    for item in roots:
        if item.IsA(UsdPhysics.Scene):continue
        for _ in Usd.PrimRange(item):
            total+=1
            if total % 2000 == 0:await app.next_update_async()
    processed=0
    for root_index,p in enumerate(roots,1):
        ext._set_status(f'공간 교체 5/8 · 건물 묶음 {root_index}/{len(roots)} ({p.GetName()}) 불러오는 중… 이 작업은 내부 진행 개수를 제공하지 않습니다.')
        await app.next_update_async()
        if p.IsA(UsdPhysics.Scene):continue
        dest=stage.DefinePrim(str(content.GetPath())+'/'+p.GetName())
        dest.GetReferences().AddReference(config['usd_path'],p.GetPath())
        ext._set_status(f'공간 교체 5/8 · 묶음 {root_index}/{len(roots)} 불러오기 완료 · 재질 연결 확인 시작')
        await app.next_update_async()
        # References to individual roots do not remap bindings to sibling Looks roots.
        # Re-author cross-root relationships/connections in the shared content namespace.
        def mapped(path):
            absolute = path if path.IsAbsolutePath() else path.MakeAbsolutePath(p.GetPath())
            return content.GetPath().AppendPath(absolute.MakeRelativePath(Sdf.Path.absoluteRootPath))
        for original in Usd.PrimRange(p):
            processed+=1
            if processed % 100 == 0 or processed == total:
                ext._set_status(f'공간 교체 5/8 · 건물 배치·재질 연결 중… 묶음 {root_index}/{len(roots)}, {processed:,}/{total:,}개 확인 ({processed/max(total,1):.0%})')
                await app.next_update_async()
            destination = stage.GetPrimAtPath(mapped(original.GetPath()))
            if not destination.IsValid():
                continue
            for rel in original.GetRelationships():
                targets = rel.GetTargets()
                if targets:
                    destination.CreateRelationship(rel.GetName(), custom=rel.IsCustom()).SetTargets([mapped(t) for t in targets])
            for attr in original.GetAttributes():
                targets = attr.GetConnections()
                if targets:
                    destination.CreateAttribute(attr.GetName(), attr.GetTypeName(), custom=attr.IsCustom()).SetConnections([mapped(t) for t in targets])
    ext._set_status('공간 교체 6/8 · 건물 크기·바닥 높이·중심 계산 중…')
    await app.next_update_async()
    up=UsdGeom.GetStageUpAxis(source)
    if up==UsdGeom.Tokens.y:UsdGeom.Xformable(content).AddRotateXOp().Set(90)
    elif up==UsdGeom.Tokens.x:UsdGeom.Xformable(content).AddRotateYOp().Set(-90)
    unit_scale=float(UsdGeom.GetStageMetersPerUnit(source))*config['scale']
    UsdGeom.Xformable(content).AddScaleOp().Set(Gf.Vec3f(unit_scale))
    # Align the room floor with Z=0 and its horizontal center with the lab, then apply user offset.
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy])
    focus=room_focus_prim(stage,room.GetPrim())
    bounds=cache.ComputeRelativeBound(focus,room.GetPrim()).ComputeAlignedRange()
    translation=Gf.Vec3d(*config['offset'])
    if not bounds.IsEmpty():
        center=bounds.GetMidpoint();translation+=Gf.Vec3d(4.-center[0],-center[1],-bounds.GetMin()[2])
    room.AddTranslateOp().Set(translation)
    ext._set_status('공간 교체 7/8 · 건물 충돌·물리 설정 준비 중…')
    await app.next_update_async()
    # Include instance descendants in the work list before expanding them.
    instance_paths=[]
    for item in Usd.PrimRange(room.GetPrim(),Usd.TraverseInstanceProxies()):
        instance_paths.append(item.GetPath())
        if len(instance_paths) % 2000 == 0:await app.next_update_async()
    instance_total=len(instance_paths)
    for index,path in enumerate(instance_paths,1):
        p=stage.GetPrimAtPath(path)
        if index % 100 == 0 or index == instance_total:
            ext._set_status(f'공간 교체 7/8 · 인스턴스 정리 · {index:,}/{instance_total:,}개 확인 ({index/max(instance_total,1):.0%})')
            await app.next_update_async()
        if p.IsValid() and p.IsInstance():p.SetInstanceable(False)
    physics_paths=[]
    for item in Usd.PrimRange(room.GetPrim()):
        physics_paths.append(item.GetPath())
        if len(physics_paths) % 2000 == 0:await app.next_update_async()
    physics_total=len(physics_paths)
    for index,path in enumerate(physics_paths,1):
        p=stage.GetPrimAtPath(path)
        if index % 100 == 0 or index == physics_total:
            ext._set_status(f'공간 교체 7/8 · 충돌·물리 설정 · {index:,}/{physics_total:,}개 처리 ({index/max(physics_total,1):.0%})')
            await app.next_update_async()
        if p.IsA(UsdPhysics.Scene):p.SetActive(False);continue
        if p.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
        if p.IsA(UsdGeom.Gprim) and not p.IsA(UsdGeom.Points) and not p.IsA(UsdGeom.BasisCurves):
            UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
            if p.IsA(UsdGeom.Mesh):UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('none')
    room.GetPrim().CreateAttribute('lab:source',Sdf.ValueTypeNames.String).Set(config['usd_path'])


def saved_directory(ext):
    return ext._extension_root().parent.parent/'saved_scenes'


def saved_rooms(ext):
    directory=saved_directory(ext)
    if not directory.exists():return []
    return [dict(file=p.name,label=p.stem) for p in sorted(directory.glob('*.usda'),key=lambda p:p.stat().st_mtime,reverse=True)]


def save_snapshot(ext,label):
    import os
    import re
    import time
    label=str(label or '수정한_방').strip()
    label=re.sub(r'[^\w가-힣 .-]', '_', label)[:60].strip('. ') or '수정한_방'
    directory=saved_directory(ext);directory.mkdir(parents=True,exist_ok=True)
    path=directory/f'{label}_{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns()%1000000:06d}.usda'
    temporary=path.with_suffix('.tmp.usda')
    # Flatten the CURRENT composed stage, not its original USD file. Includes added/deleted prims,
    # edits in the session layer, references, current placements and custom lab attributes.
    flattened=ext._stage.Flatten()
    metadata=dict(flattened.customLayerData)
    metadata.update(interactivePhysicsLabSnapshot=1,roomLabel=label,roomSource=str(getattr(ext,"_room_source","")))
    target=getattr(ext,"_teaching_camera_target",None)
    if target is not None:metadata["teachingTarget"]=Gf.Vec3d(*target)
    flattened.customLayerData=metadata
    if not flattened.Export(str(temporary)):raise ValueError('장면 파일을 저장하지 못했습니다.')
    os.replace(temporary,path)
    return path


def snapshot_config(ext,filename):
    directory=saved_directory(ext).resolve()
    path=(directory/str(filename)).resolve()
    if path.parent!=directory or path.suffix!='.usda' or not path.is_file():
        raise ValueError('저장된 장면 목록에서 파일을 선택하세요.')
    source=Usd.Stage.Open(str(path))
    if not source or source.GetRootLayer().customLayerData.get('interactivePhysicsLabSnapshot')!=1:
        raise ValueError('현재 장면 저장으로 만든 파일이 아닙니다.')
    if not source.GetPrimAtPath('/World/InteractivePhysicsLab').IsValid():raise ValueError('실습 장면 데이터가 없습니다.')
    return dict(kind='saved',snapshot_path=str(path),include_lab=True,usd_path='')


def relocate_snapshot_assets(layer, asset_root=None):
    """Repair missing bundled assets after moving a saved project to another folder.

    Only the anonymous flattened snapshot is edited; the user's saved file and
    existing external assets are left intact.
    """
    asset_root = (Path(asset_root) if asset_root is not None else
                  Path(__file__).resolve().parent.parent/'assets').resolve()
    marker = '/exts/edu.interactive.physics/assets/'
    snapshot = Usd.Stage.Open(layer)
    changed = 0

    def relocated(value):
        if not isinstance(value, Sdf.AssetPath) or not value.path:
            return value
        original = Path(value.path)
        if not original.is_absolute() or original.is_file():
            return value
        if value.resolvedPath and Path(value.resolvedPath).is_file():
            return value
        if marker not in value.path:
            return value
        candidate = (asset_root/value.path.split(marker, 1)[1]).resolve()
        if not candidate.is_relative_to(asset_root) or not candidate.is_file():
            return value
        return Sdf.AssetPath(candidate.as_posix())

    for prim in snapshot.Traverse():
        for attr in prim.GetAttributes():
            kind = attr.GetTypeName()
            if kind not in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
                continue
            for sample in [Usd.TimeCode.Default(), *attr.GetTimeSamples()]:
                old = attr.Get(sample)
                if old is None:
                    continue
                new = relocated(old) if kind == Sdf.ValueTypeNames.Asset else [relocated(v) for v in old]
                if new != old:
                    attr.Set(new, sample)
                    changed += 1
    return changed


def wall_logo_asset(asset_root=None):
    root = Path(asset_root) if asset_root is not None else Path(__file__).resolve().parent.parent/'assets'
    image = root/'branding/custom_wall_logo.png'
    if not image.is_file():
        return None
    with image.open('rb') as stream:
        header = stream.read(24)
    if len(header) != 24 or header[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('Wall logo must be a PNG image')
    width, height = struct.unpack('>II', header[16:24])
    if not width or not height:
        raise ValueError('Invalid wall logo dimensions')
    return image.resolve(), width, height


def apply_wall_logo(stage, asset_root=None):
    """Map the supplied image to the existing three wall signs and floor sign."""
    image = wall_logo_asset(asset_root)
    if image is None:
        return 0
    path, image_width, image_height = image
    count = 0
    for name in ('GISTWallSign', 'GISTWallSignLeft', 'GISTWallSignBack', 'GISTFloorSign'):
        base = '/World/InteractivePhysicsLab/Branding/'+name
        mesh = UsdGeom.Mesh.Get(stage, base+'/LogoMesh')
        texture = stage.GetPrimAtPath(base+'/LogoMaterial/Texture')
        if not mesh or not texture.IsValid():
            continue
        points = mesh.GetPointsAttr().Get()
        if points is None or len(points) != 4:
            continue
        horizontal = points[1]-points[0]
        vertical = points[3]-points[0]
        if horizontal.GetLength() <= 0 or vertical.GetLength() <= 0:
            continue
        center = (points[0]+points[2])*0.5
        horizontal *= 0.5
        vertical = vertical.GetNormalized()*(horizontal.GetLength()*image_height/image_width)
        updated = [center-horizontal-vertical, center+horizontal-vertical,
                   center+horizontal+vertical, center-horizontal+vertical]
        mesh.GetPointsAttr().Set(updated)
        mesh.CreateExtentAttr([Gf.Vec3f(*(min(p[i] for p in updated) for i in range(3))),
                               Gf.Vec3f(*(max(p[i] for p in updated) for i in range(3)))])
        texture.GetAttribute('inputs:file').Set(Sdf.AssetPath(path.as_posix()))
        stage.GetPrimAtPath(base).CreateAttribute('lab:source', Sdf.ValueTypeNames.String).Set('assets/branding/custom_wall_logo.png')
        count += 1
    return count


def restore_snapshot(stage,path):
    source=Usd.Stage.Open(str(path))
    flattened=source.Flatten()
    relocate_snapshot_assets(flattened)
    # Transfer the complete saved state, including removed objects. Do not regenerate CSV objects.
    stage.GetSessionLayer().Clear()
    stage.SetEditTarget(stage.GetRootLayer())
    stage.GetRootLayer().TransferContent(flattened)
    apply_wall_logo(stage)
    return stage.GetPrimAtPath('/World/InteractivePhysicsLab')


def room_library(ext, purpose="room"):
    project=ext._extension_root().parents[1]
    items=[]
    folders = (project/'sample_data', project/'room_assets', project/'.web_runtime/room_assets') if purpose == 'room' else (project/'object_assets', project/'.web_runtime/object_assets')
    for folder in folders:
        if not folder.is_dir():continue
        for path in sorted(folder.rglob('*')):
            if not any(part.endswith('.pending') for part in path.parts) and path.is_file() and path.suffix.lower() in ('.usd','.usda','.usdc','.usdz'):
                label=path.stem
                try:
                    import json
                    label=json.loads(path.with_suffix(path.suffix+'.json').read_text()).get('label') or label
                except (OSError,ValueError):pass
                items.append({'label':label,'path':str(path.resolve())})
                if len(items)>=200:return items
    return items
