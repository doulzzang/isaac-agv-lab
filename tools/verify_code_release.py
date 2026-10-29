#!/usr/bin/env python3
"""Verify actual release installation offline using current and previously published updaters."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
from unittest.mock import patch
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    release = ROOT/'release'
    pair = json.loads((release/'LATEST_RELEASE_PAIR.json').read_text())
    archive = release/pair['browser_zip']
    latest = json.loads(archive.with_suffix('.zip.update.json').read_text())
    checks = {}
    for key in ('browser_zip', 'homepage_zip'):
        path = release/pair[key]
        with path.open('rb') as stream:
            digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        assert digest == path.with_suffix('.zip.sha256').read_text().split()[0]
        with zipfile.ZipFile(path) as z:
            assert z.testzip() is None
        checks[key] = dict(sha256=digest, crc_ok=True, bytes=path.stat().st_size)
    current = load('current_release_updater', ROOT/'tools/startup_update.py')
    legacy = types.ModuleType('legacy_release_updater')
    legacy.__file__ = str(ROOT/'tools/startup_update.py')
    with zipfile.ZipFile(release/'archive_20261029'/pair['browser_zip']) as z:
        code = z.read('isaac_interactive_physics_lab/tools/startup_update.py')
    exec(compile(code, legacy.__file__, 'exec'), legacy.__dict__)
    results = []
    for label, updater in [('current', current), ('previously_published', legacy)]:
        with tempfile.TemporaryDirectory(prefix='isaac-release-install-') as folder:
            root = Path(folder)
            (root/'INSTALL_TARGET.json').write_text(json.dumps(latest['target']))
            (root/'RELEASE_MANIFEST.json').write_text(json.dumps({'build':'20260917_211045'}))
            saved = ROOT/'saved_scenes/편집한_공간_20260916_044353_085475.usda'
            sentinels = {'saved_scenes/user.usda': saved.read_bytes() if saved.is_file() else b'user scene',
                         '.web_runtime/homepage.json': b'private registration',
                         '.web_runtime/startup_scene.json': b'{"file":"user.usda"}',
                         'web_app/uploads/user.usda': b'user upload',
                         '.install/isaac_sim_root.txt': b'user installation'}
            for name, data in sentinels.items():
                path = root/name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
            with zipfile.ZipFile(archive) as z:
                (root/'README.md').write_bytes(z.read('isaac_interactive_physics_lab/README.md'))
            (root/'README.md').chmod(0o644)
            before = (root/'README.md').stat()
            with patch.object(updater, 'fetch', return_value=json.dumps(latest).encode()), patch.object(updater.urllib.request, 'urlopen', side_effect=lambda *a, **kw: archive.open('rb')):
                assert updater.update(root)
            for name, data in sentinels.items():
                assert (root/name).read_bytes() == data, name
            if label == 'current':
                after = (root/'README.md').stat()
                assert (before.st_ino, before.st_mtime_ns) == (after.st_ino, after.st_mtime_ns)
            assert 'version = "3.5.2"' in (root/'exts/edu.interactive.physics/config/extension.toml').read_text()
            asset = root/'exts/edu.interactive.physics/assets/branding/custom_wall_logo.png'
            assert asset.read_bytes() == (ROOT/'exts/edu.interactive.physics/assets/branding/custom_wall_logo.png').read_bytes()
            result = dict(updater=label, passed=True, user_state_preserved=list(sentinels))
            if saved.is_file():
                from pxr import Usd, UsdGeom
                room = load('installed_room_'+label, root/'exts/edu.interactive.physics/edu_interactive_physics/room_scene.py')
                stage = Usd.Stage.CreateInMemory()
                room.restore_snapshot(stage, root/'saved_scenes/user.usda')
                for sign in ('GISTWallSign', 'GISTWallSignLeft', 'GISTWallSignBack', 'GISTFloorSign'):
                    base = '/World/InteractivePhysicsLab/Branding/'+sign
                    actual = stage.GetPrimAtPath(base+'/LogoMaterial/Texture').GetAttribute('inputs:file').Get()
                    assert actual.path == str(asset)
                    points = UsdGeom.Mesh.Get(stage, base+'/LogoMesh').GetPointsAttr().Get()
                    ratio = (points[3]-points[0]).GetLength()/(points[1]-points[0]).GetLength()
                    assert abs(ratio-66/282) < 1e-5
                assert (root/'saved_scenes/user.usda').read_bytes() == sentinels['saved_scenes/user.usda']
                result['restored_wall_floor_logos'] = 4
            results.append(result)
    checks.update(build=latest['build'], version='3.5.2', actual_package_updates=results,
                  transport='Local package supplied through mocked download; no remote or renderer access')
    output = release/'PACKAGE_VERIFICATION_20261029.json'
    output.write_text(json.dumps(checks, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(checks, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
