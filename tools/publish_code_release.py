#!/usr/bin/env python3
"""Publish the reviewed code release from a normal terminal after explicit approval."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT / 'release/PUBLISH_PLAN_20261029.json'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def run(args, cwd=None, capture=False):
    result = subprocess.run(args, cwd=cwd, check=True, text=True,
                            stdout=subprocess.PIPE if capture else None)
    return result.stdout.strip() if capture else None


def checked_path(name):
    relative = Path(name)
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('Unsafe publication path: ' + name)
    path = ROOT / relative
    if path.is_symlink() or not path.is_file():
        raise ValueError('Missing reviewed file: ' + name)
    return path


def verify(plan):
    for name, expected in plan['reviewed_files'].items():
        if digest(checked_path(name)) != expected:
            raise ValueError('File changed after review; rebuild before publishing: ' + name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--approve-code-release', action='store_true',
                        help='Explicitly approve Git commit/push and replacement of the reviewed GitHub assets')
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    plan = json.loads(PLAN.read_text())
    if plan.get('publication_blocked'):
        raise RuntimeError('Release must be rebuilt and reviewed: ' + plan['publication_blocked'])
    verify(plan)
    if args.check_only:
        print('Reviewed source and release assets: hashes OK; no Git or remote changes')
        return
    if not args.approve_code_release:
        parser.error('Approval required: --approve-code-release')
    if not shutil.which('gh') or not shutil.which('git'):
        raise RuntimeError('git and authenticated GitHub CLI (gh) are required')
    run(['gh', 'auth', 'status'])
    receipt_path = ROOT / 'release/PUBLICATION_RECEIPT_20261029.json'
    receipt = dict(version=plan['version'], build=plan['build'], status='preflight',
                   started_at=datetime.datetime.now().astimezone().isoformat(),
                   homepage_published=False, uploaded_assets=[])
    def save():
        receipt_path.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')
    try:
        with tempfile.TemporaryDirectory(prefix='isaac-code-publish-') as folder:
            checkout = Path(folder) / 'repo'
            run(['gh', 'repo', 'clone', plan['repository'], str(checkout)])
            head = run(['git', 'rev-parse', 'HEAD'], checkout, capture=True)
            if head != plan['expected_remote_head']:
                raise RuntimeError('Remote branch changed since the known release. No commit, push, or asset replacement performed. Review remote HEAD: ' + head)
            branch = run(['git', 'symbolic-ref', '--short', 'HEAD'], checkout, capture=True)
            verify(plan)
            for name in plan['git_files']:
                source = checked_path(name)
                destination = checkout / name
                if any(parent.is_symlink() for parent in destination.parents if parent != checkout.parent):
                    raise ValueError('Checkout contains a symlink parent: ' + name)
                if destination.is_symlink():
                    raise ValueError('Checkout contains a symlink: ' + name)
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination)
            run(['git', 'add', '--', *plan['git_files']], checkout)
            changed = subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=checkout).returncode
            if changed == 1:
                run(['git', 'diff', '--cached', '--stat'], checkout)
                run(['git', 'commit', '-m', 'Release 3.5.2: restore USB humidity display and code updates'], checkout)
                run(['git', 'push', 'origin', 'HEAD:' + branch], checkout)
            elif changed != 0:
                raise RuntimeError('Could not inspect staged changes')
            receipt.update(status='git_pushed', github_commit=run(['git', 'rev-parse', 'HEAD'], checkout, capture=True))
            save()
            for name in plan['assets_before_metadata'] + [plan['update_metadata']]:
                verify(plan)
                run(['gh', 'release', 'upload', plan['tag'], str(ROOT / name), '--repo', plan['repository'], '--clobber'])
                receipt['uploaded_assets'].append(name)
                save()
            assets = json.loads(run(['gh', 'api', 'repos/' + plan['repository'] + '/releases/tags/' + plan['tag']], capture=True))['assets']
            by_name = {asset['name']: asset for asset in assets}
            for name in plan['assets_before_metadata'] + [plan['update_metadata']]:
                expected = plan['reviewed_files'][name]
                asset = by_name.get(Path(name).name)
                if asset is None:
                    raise RuntimeError('Remote asset missing: ' + name)
                confirmed = asset.get('digest') == 'sha256:' + expected
                if not asset.get('digest'):
                    download = Path(folder) / 'verify'
                    download.mkdir(exist_ok=True)
                    run(['gh', 'release', 'download', plan['tag'], '--repo', plan['repository'],
                         '--pattern', Path(name).name, '--dir', str(download), '--clobber'])
                    confirmed = digest(download / Path(name).name) == expected
                if not confirmed:
                    raise RuntimeError('Remote asset SHA256 not confirmed: ' + name)
            pair_path = ROOT / 'release/LATEST_RELEASE_PAIR.json'
            pair = json.loads(pair_path.read_text())
            pair.update(published=True, publication=dict(receipt, status='succeeded', assets_sha256_verified=True))
            pair_path.write_text(json.dumps(pair, ensure_ascii=False, indent=2) + '\n')
            receipt.update(status='succeeded', assets_sha256_verified=True,
                           completed_at=datetime.datetime.now().astimezone().isoformat())
            save()
            print('Code release published and remote SHA256 verified. Homepage deployment was not performed.')
    except Exception as error:
        receipt.update(status='incomplete', error=str(error))
        save()
        raise


if __name__ == '__main__':
    main()
