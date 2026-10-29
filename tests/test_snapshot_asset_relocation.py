"""Run with Isaac Sim's USD Python libraries; no renderer or scene rebuild."""
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
from pxr import Sdf, Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('snapshot_room', ROOT/'exts/edu.interactive.physics/edu_interactive_physics/room_scene.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class SnapshotAssetTests(unittest.TestCase):
    def test_only_missing_bundled_assets_are_relocated(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            assets = root/'assets'
            (assets/'branding').mkdir(parents=True)
            logo = assets/'branding/gist_wall_logo.png'
            logo.write_bytes(b'logo')
            existing = root/'existing.png'
            existing.write_bytes(b'custom')
            stage = Usd.Stage.CreateInMemory()
            prim = stage.DefinePrim('/Material')
            old = '/missing-project/exts/edu.interactive.physics/assets/branding/gist_wall_logo.png'
            paths = {'missing':old, 'existing':str(existing), 'external':'/other/custom.png',
                     'unavailable':'/missing-project/exts/edu.interactive.physics/assets/missing.png',
                     'escape':'/missing-project/exts/edu.interactive.physics/assets/../existing.png'}
            for key, value in paths.items():
                prim.CreateAttribute(key,Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(value))
            prim.CreateAttribute('array',Sdf.ValueTypeNames.AssetArray).Set([Sdf.AssetPath(old),Sdf.AssetPath(str(existing))])
            prim.CreateAttribute('animated',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(old),1)
            self.assertEqual(m.relocate_snapshot_assets(stage.GetRootLayer(), assets),3)
            self.assertEqual(prim.GetAttribute('missing').Get().path,str(logo))
            self.assertEqual(prim.GetAttribute('array').Get()[0].path,str(logo))
            self.assertEqual(prim.GetAttribute('animated').Get(1).path,str(logo))
            for key in ('existing','external','unavailable','escape'):
                self.assertEqual(prim.GetAttribute(key).Get().path,paths[key])
            self.assertEqual(m.relocate_snapshot_assets(stage.GetRootLayer(), assets),0)

    def test_restore_repairs_logo_without_replacing_layout_or_saved_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'scene.usda'
            source = Usd.Stage.CreateNew(str(path))
            UsdGeom.Xform.Define(source,'/World/InteractivePhysicsLab')
            cube=UsdGeom.Cube.Define(source,'/World/InteractivePhysicsLab/MyObject')
            cube.AddTranslateOp().Set((1,2,3))
            attr=cube.GetPrim().CreateAttribute('logo',Sdf.ValueTypeNames.Asset)
            attr.Set(Sdf.AssetPath('/missing-project/exts/edu.interactive.physics/assets/branding/gist_wall_logo.png'))
            source.GetRootLayer().Save()
            before=path.read_bytes()
            target=Usd.Stage.CreateInMemory()
            target.DefinePrim('/Removed')
            self.assertTrue(m.restore_snapshot(target,path).IsValid())
            restored=target.GetPrimAtPath('/World/InteractivePhysicsLab/MyObject')
            self.assertEqual(tuple(restored.GetAttribute('xformOp:translate').Get()),(1,2,3))
            self.assertTrue(Path(restored.GetAttribute('logo').Get().path).is_file())
            self.assertFalse(target.GetPrimAtPath('/Removed').IsValid())
            self.assertEqual(path.read_bytes(),before)

    def test_actual_saved_scene_wall_and_floor_logo_paths(self):
        path=ROOT/'saved_scenes/편집한_공간_20260916_044353_085475.usda'
        if not path.is_file():self.skipTest('Local saved scene is not part of portable release')
        before=hashlib.sha256(path.read_bytes()).hexdigest()
        stage=Usd.Stage.CreateInMemory()
        m.restore_snapshot(stage,path)
        base='/World/InteractivePhysicsLab/Branding/'
        for name in ('GISTWallSign','GISTWallSignLeft','GISTWallSignBack','GISTFloorSign'):
            prim=stage.GetPrimAtPath(base+name+'/LogoMaterial/Texture')
            with self.subTest(sign=name):
                self.assertTrue(prim.IsValid())
                asset=prim.GetAttribute('inputs:file').Get()
                self.assertEqual(Path(asset.path),ROOT/'exts/edu.interactive.physics/assets/branding/custom_wall_logo.png')
                self.assertTrue(Path(asset.path).is_file())
                mesh=UsdGeom.Mesh.Get(stage,base+name+'/LogoMesh')
                points=mesh.GetPointsAttr().Get()
                width=(points[1]-points[0]).GetLength()
                height=(points[3]-points[0]).GetLength()
                self.assertAlmostEqual(height/width,66/282,places=5)
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),before)

    def test_new_scene_uses_supplied_logo_on_wall_and_floor(self):
        import ast
        from types import SimpleNamespace
        from pxr import Gf, UsdShade
        tree=ast.parse((ROOT/'exts/edu.interactive.physics/edu_interactive_physics/extension.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_create_gist_sign_on_wall')
        env=dict(UsdGeom=UsdGeom,UsdShade=UsdShade,Sdf=Sdf,Gf=Gf,
                 LAB_ROOT='/World/InteractivePhysicsLab',wall_logo_asset=m.wall_logo_asset,
                 carb=SimpleNamespace(log_info=lambda *a:None,log_warn=lambda *a:None))
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual sign creation>','exec'),env)
        stage=Usd.Stage.CreateInMemory()
        owner=SimpleNamespace(_stage=stage,_extension_root=lambda:ROOT/'exts/edu.interactive.physics')
        for name,up in [('GISTWallSign',(0,0,1)),('GISTFloorSign',(1,0,0))]:
            path='/World/InteractivePhysicsLab/Branding/'+name
            self.assertTrue(env[fn.name](owner,path,(0,0,0),(0,-1,0),up=up))
            texture=stage.GetPrimAtPath(path+'/LogoMaterial/Texture').GetAttribute('inputs:file').Get()
            self.assertEqual(Path(texture.path).name,'custom_wall_logo.png')
            points=UsdGeom.Mesh.Get(stage,path+'/LogoMesh').GetPointsAttr().Get()
            self.assertAlmostEqual((points[3]-points[0]).GetLength()/(points[1]-points[0]).GetLength(),66/282,places=5)

if __name__ == '__main__':unittest.main()
