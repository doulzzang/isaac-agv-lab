import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('release_pair', Path(__file__).resolve().parents[1] / 'tools/build_release_pair.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / 'page.html').write_bytes(b'verified source')
        self.manifest = {'base_commit': 'a' * 40, 'files': {'page.html': hashlib.sha256(b'verified source').hexdigest()}}
        self.save()
        self.probe = patch.object(m.subprocess, 'run', return_value=subprocess.CompletedProcess([], 128, '', 'no repository'))
        self.probe.start()
        self.addCleanup(self.probe.stop)

    def save(self):
        (self.root / 'SNAPSHOT_MANIFEST.json').write_text(json.dumps(self.manifest))

    def test_verified_snapshot_preserves_provenance(self):
        names, commit, source = m.homepage_sources(self.root)
        self.assertEqual(names, ['page.html'])
        self.assertEqual(commit, 'a' * 40)
        self.assertIn('archived snapshot', source)

    def test_changed_source_rejected_before_package_build(self):
        (self.root / 'page.html').write_text('unverified change')
        with self.assertRaisesRegex(ValueError, 'hash mismatch'):
            m.homepage_sources(self.root)

    def test_missing_source_rejected(self):
        (self.root / 'page.html').unlink()
        with self.assertRaisesRegex(ValueError, 'Missing'):
            m.homepage_sources(self.root)

    def test_path_escape_rejected(self):
        self.manifest['files'] = {'../outside.html': 'x'}
        self.save()
        with self.assertRaisesRegex(ValueError, 'Unsafe'):
            m.homepage_sources(self.root)

if __name__ == '__main__':
    unittest.main()
