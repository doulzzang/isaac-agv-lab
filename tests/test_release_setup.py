import importlib.util
import os
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,ROOT/path)
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value)
    return value

setup=module('release_setup','tools/setup/install.py')
build=module('release_build','tools/build_release.py')
learned=module('release_learned','tools/setup/prepare_learned.py')

class ReleaseTests(unittest.TestCase):
    def test_saved_configuration_and_spaces(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)/'project';root.mkdir()
            isaac=Path(directory)/'Isaac Sim';isaac.mkdir()
            (isaac/'isaac-sim.sh').touch();(isaac/'VERSION').write_text('4.5.0')
            (root/'.install').mkdir();(root/'.install/isaac_sim_root.txt').write_text(str(isaac)+'\n')
            (root/'INSTALL_TARGET.json').write_text(json.dumps(dict(isaac='4.5')))
            with patch.dict(os.environ,{},clear=True):
                self.assertEqual(setup.resolve_isaac(root=root),isaac)
            (isaac/'VERSION').write_text('5.0.0')
            with self.assertRaises(ValueError):setup.resolve_isaac(str(isaac),root)

    def test_relocation_and_missing_asset(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'saved_scenes').mkdir();(root/'exts').mkdir()
            asset=root/'exts/logo.png';asset.write_bytes(b'image')
            source=root/'saved_scenes/room.usda'
            data,count=build.portable_usda(f'asset x = @{asset}@'.encode(),source,root)
            self.assertEqual(data,b'asset x = @../exts/logo.png@');self.assertEqual(count,1)
            asset.unlink()
            with self.assertRaises(ValueError):build.portable_usda(f'@{asset}@'.encode(),source,root)
            with self.assertRaises(ValueError):build.portable_usda(b'@/outside/file.png@',source,root)

    def test_ubuntu24_profile_and_isaac51(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'INSTALL_TARGET.json').write_text(json.dumps(dict(ubuntu='24.04',isaac='5.1',system_python=[3,12])))
            isaac=root/'Isaac 5.1';isaac.mkdir()
            (isaac/'isaac-sim.sh').touch();(isaac/'VERSION').write_text('5.1.0-rc')
            self.assertEqual(setup.resolve_isaac(str(isaac),root),isaac)
            with patch.object(setup,'profile',return_value=setup.profile(root)), patch.object(setup.platform,'system',return_value='Linux'), patch.object(setup.platform,'machine',return_value='aarch64'), patch.object(setup.Path,'read_text',return_value='ID=ubuntu\nVERSION_ID="24.04"'), patch.object(setup.sys,'version_info',(3,12)), patch.object(setup,'run') as run:
                setup.validate_host()
                run.assert_called_once()
            (isaac/'VERSION').write_text('4.5.0')
            with self.assertRaises(ValueError):setup.resolve_isaac(str(isaac),root)

    def test_nomad_isolated_python_on_ubuntu24(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            binary=root/'.nav2_runtime/bin/micromamba';binary.parent.mkdir(parents=True);binary.touch()
            with patch.object(learned.sys,'version_info',(3,12)), patch.object(learned.platform,'machine',return_value='aarch64'), patch.object(learned.subprocess,'run') as run:
                python=learned.prepare(root)
                command=run.call_args.args[0]
                self.assertEqual(command,[learned.sys.executable,'-m','venv',str(root/'.learned_runtime/env')])
                self.assertEqual(python,root/'.learned_runtime/env/bin/python')

    def test_release_excludes_machine_state(self):
        paths=[p.relative_to(ROOT).as_posix() for p in build.inputs()]
        self.assertIn('requirements.txt',paths)
        self.assertIn('install.sh',paths)
        self.assertFalse(any(p.startswith(('.learned_runtime/','.nav2_runtime/','release/')) or '__pycache__' in p for p in paths))

if __name__=='__main__':unittest.main()
