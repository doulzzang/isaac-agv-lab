import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import sys
import unittest
from unittest.mock import patch
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
spec = importlib.util.spec_from_file_location('startup_update', Path(__file__).resolve().parents[1]/'tools/startup_update.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
TARGET = {'isaac': '5.1', 'architecture': 'aarch64'}


class UpdateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root/'INSTALL_TARGET.json').write_text(json.dumps(TARGET))
        (self.root/'RELEASE_MANIFEST.json').write_text(json.dumps({'build': '20260913_154402'}))
        (self.root/'run_isaac_with_browser.sh').write_text('old launcher')
        (self.root/'.web_runtime').mkdir()
        (self.root/'.web_runtime/homepage.json').write_text('private-token')
        (self.root/'saved_scenes').mkdir()
        (self.root/'saved_scenes/user.usda').write_text('user scene')

    def release(self, extra=None, target=TARGET):
        files = {'run_isaac_with_browser.sh': b'new launcher', 'tools/startup_update.py': b'new updater',
                 'INSTALL_TARGET.json': json.dumps(target).encode()}
        files.update(extra or {})
        manifest = {'build': '20260916_123456', 'target': target,
                    'files': {k: hashlib.sha256(v).hexdigest() for k, v in files.items()}}
        data = io.BytesIO()
        with zipfile.ZipFile(data, 'w') as archive:
            for name, content in files.items():
                archive.writestr('isaac_interactive_physics_lab/'+name, content)
            archive.writestr('isaac_interactive_physics_lab/RELEASE_MANIFEST.json', json.dumps(manifest))
        blob = data.getvalue()
        latest = {'build': manifest['build'], 'target': target, 'asset': m.ASSET,
                  'bytes': len(blob), 'sha256': hashlib.sha256(blob).hexdigest()}
        return latest, blob

    def run_update(self, latest, blob):
        with patch.object(m, 'fetch', return_value=json.dumps(latest).encode()), \
             patch.object(m.urllib.request, 'urlopen', side_effect=lambda *a, **kw: io.BytesIO(blob)):
            return m.update(self.root)

    def test_verified_update_preserves_registration_scenes_and_executable(self):
        self.assertTrue(self.run_update(*self.release()))
        self.assertEqual((self.root/'run_isaac_with_browser.sh').read_text(), 'new launcher')
        self.assertTrue((self.root/'run_isaac_with_browser.sh').stat().st_mode & 0o100)
        self.assertEqual((self.root/'.web_runtime/homepage.json').read_text(), 'private-token')
        self.assertEqual((self.root/'saved_scenes/user.usda').read_text(), 'user scene')
        self.assertFalse((self.root/'.install/update-transaction').exists())

    def test_current_and_older_releases_do_not_download(self):
        latest, blob = self.release()
        for build in ['20260913_154402', '20260912_154402']:
            latest['build'] = build
            with patch.object(m, 'fetch', return_value=json.dumps(latest).encode()), \
                 patch.object(m.urllib.request, 'urlopen', side_effect=AssertionError('must not download')):
                self.assertFalse(m.update(self.root))

    def test_identical_files_keep_inode_and_timestamp(self):
        launcher = self.root/'run_isaac_with_browser.sh'
        launcher.write_text('new launcher')
        launcher.chmod(0o755)
        before = launcher.stat()
        self.assertTrue(self.run_update(*self.release()))
        after = launcher.stat()
        self.assertEqual((after.st_ino, after.st_mtime_ns), (before.st_ino, before.st_mtime_ns))

    def test_permission_changes_are_still_applied(self):
        launcher = self.root/'run_isaac_with_browser.sh'
        launcher.write_text('new launcher')
        launcher.chmod(0o644)
        self.assertTrue(self.run_update(*self.release()))
        self.assertTrue(launcher.stat().st_mode & 0o100)

    def test_bad_checksum_leaves_original(self):
        latest, blob = self.release()
        latest['sha256'] = '0'*64
        with self.assertRaisesRegex(ValueError, '검증 실패'):
            self.run_update(latest, blob)
        self.assertEqual((self.root/'run_isaac_with_browser.sh').read_text(), 'old launcher')

    def test_network_failure_leaves_original(self):
        with patch.object(m, 'fetch', side_effect=OSError('offline')):
            with self.assertRaises(OSError):
                m.update(self.root)
        self.assertEqual((self.root/'run_isaac_with_browser.sh').read_text(), 'old launcher')

    def test_forbidden_paths_cannot_write(self):
        for name in ['../outside', '/tmp/outside', '.web_runtime/homepage.json', 'saved_scenes/user.usda', 'exts/../../outside']:
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.run_update(*self.release({name: b'bad'}))
        self.assertEqual((self.root/'.web_runtime/homepage.json').read_text(), 'private-token')

    def test_wrong_target_cannot_install(self):
        with self.assertRaisesRegex(ValueError, '대상'):
            self.run_update(*self.release(target={'isaac': '6.0', 'architecture': 'aarch64'}))

    def test_symlink_destination_cannot_escape(self):
        outside = self.root/'outside'
        outside.mkdir()
        (self.root/'tools').symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, '심볼릭'):
            self.run_update(*self.release())
        self.assertFalse((outside/'startup_update.py').exists())

    def test_mid_apply_failure_rolls_back_all_files(self):
        original = m.atomic_copy
        calls = []
        def fail_once(source, dest):
            if 'files' in source.parts:
                calls.append(dest)
                if len(calls) == 2:
                    raise OSError('disk write failure')
            original(source, dest)
        with patch.object(m, 'atomic_copy', side_effect=fail_once), self.assertRaises(OSError):
            self.run_update(*self.release())
        self.assertEqual((self.root/'run_isaac_with_browser.sh').read_text(), 'old launcher')
        self.assertEqual(json.loads((self.root/'RELEASE_MANIFEST.json').read_text())['build'], '20260913_154402')

    def test_crash_journal_recovers_on_next_launch(self):
        work = self.root/'.install/update-transaction'
        (work/'backup').mkdir(parents=True)
        (work/'backup/run_isaac_with_browser.sh').write_text('original')
        (self.root/'new.md').write_text('partial new file')
        (work/'journal.json').write_text(json.dumps([
            {'name': 'run_isaac_with_browser.sh', 'existed': True},
            {'name': 'new.md', 'existed': False}]))
        m.recover(self.root)
        self.assertEqual((self.root/'run_isaac_with_browser.sh').read_text(), 'original')
        self.assertFalse((self.root/'new.md').exists())

    def test_zenity_extra_button_label_wins_over_exit_code(self):
        class Process:
            returncode = 1
            def communicate(self): return ('그냥 실행\n', '')
        with patch.object(m, 'start_dialog', return_value=Process()):
            self.assertEqual(m.dialog('x', '업데이트 확인', ('그냥 실행', '종료')), '그냥 실행')

    def test_skip_choice_does_not_contact_server(self):
        with patch.object(m, 'ROOT', self.root), patch.object(m, 'dialog', return_value='그냥 실행'), \
             patch.dict(m.os.environ, {'LAB_SKIP_UPDATE_ONCE': ''}), patch.object(m, 'update') as update:
            self.assertEqual(m.main(), 0)
            update.assert_not_called()

    def test_updated_launch_returns_restart_code(self):
        with patch.object(m, 'ROOT', self.root), patch.object(m, 'dialog', side_effect=['업데이트 확인', '실행']), \
             patch.dict(m.os.environ, {'DISPLAY': '', 'WAYLAND_DISPLAY': '', 'LAB_SKIP_UPDATE_ONCE': ''}), \
             patch.object(m, 'update', return_value=True):
            self.assertEqual(m.main(), 20)


if __name__ == '__main__':
    unittest.main()
