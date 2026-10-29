import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('publish_code_release', Path(__file__).resolve().parents[1] / 'tools/publish_code_release.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class PublishPreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / 'release').mkdir()
        (self.root / 'code.py').write_bytes(b'reviewed')
        self.plan = dict(version='3.5.2', build='20261029_160000', repository='owner/repo',
                         expected_remote_head='known', reviewed_files={'code.py': hashlib.sha256(b'reviewed').hexdigest()})
        self.plan_path = self.root / 'release/plan.json'
        self.plan_path.write_text(json.dumps(self.plan))
        self.root_patch = patch.object(m, 'ROOT', self.root)
        self.root_patch.start()
        self.addCleanup(self.root_patch.stop)
        self.plan_patch = patch.object(m, 'PLAN', self.plan_path)
        self.plan_patch.start()
        self.addCleanup(self.plan_patch.stop)

    def test_modified_source_rejected(self):
        (self.root / 'code.py').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'changed after review'):
            m.verify(self.plan)

    def test_check_only_does_not_use_git_or_github(self):
        with patch('sys.argv', ['publish', '--check-only']), patch.object(m, 'run', side_effect=AssertionError('unexpected external command')):
            m.main()

    def test_remote_advance_blocks_commit_push_and_upload(self):
        commands = []
        def fake_run(args, cwd=None, **kwargs):
            commands.append(args)
            if args[:3] == ['git', 'rev-parse', 'HEAD']:
                return 'new-remote-head'
            return ''
        with patch('sys.argv', ['publish', '--approve-code-release']), patch.object(m.shutil, 'which', return_value='/usr/bin/fake'), patch.object(m, 'run', side_effect=fake_run):
            with self.assertRaisesRegex(RuntimeError, 'Remote branch changed'):
                m.main()
        self.assertFalse(any('push' in c or 'commit' in c or 'upload' in c for c in commands))

if __name__ == '__main__':
    unittest.main()
