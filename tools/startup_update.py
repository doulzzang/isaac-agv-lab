#!/usr/bin/env python3
"""Pre-launch release updater. Called while the launcher's exclusive lock is held."""
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
BASE = 'https://github.com/doulzzang/isaac-agv-lab/releases/download/v2026.09.13-spark/'
ASSET = 'isaac_interactive_physics_lab_spark_isaac51_20260913_154402.zip'
FOLDERS = {'web_app', 'exts', 'tools', 'tests', 'sample_data', 'room_assets',
           'homepage_update', 'presentation_handoff', 'validation_artifacts'}
MAX_DOWNLOAD = 2 * 1024**3


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def safe_path(name):
    p = PurePosixPath(name)
    if not name or '\\' in name or p.is_absolute() or '..' in p.parts or str(p) != name:
        raise ValueError('잘못된 업데이트 경로')
    if any(part.startswith('.') for part in p.parts):
        raise ValueError('개인 설정 경로는 업데이트할 수 없습니다.')
    if len(p.parts) == 1:
        if p.suffix not in {'.sh', '.md', '.yml'} and name not in {
                'requirements.txt', 'requirement.txt', 'INSTALL_TARGET.json', 'RELEASE_MANIFEST.json'}:
            raise ValueError('허용하지 않은 업데이트 파일')
    elif p.parts[0] not in FOLDERS or any(x in {'uploads', 'camera', '__pycache__'} for x in p.parts):
        raise ValueError('사용자 자료 경로는 업데이트할 수 없습니다.')
    return p


def destination(root, name):
    relative = safe_path(name)
    path = root
    for part in relative.parts:
        path = path / part
        if path.is_symlink():
            raise ValueError('심볼릭 링크 경로는 업데이트할 수 없습니다: ' + name)
    return path


def atomic_copy(source, dest):
    dest.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.update-', dir=dest.parent)
    try:
        with os.fdopen(fd, 'wb') as stream, source.open('rb') as incoming:
            shutil.copyfileobj(incoming, stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(name, source.stat().st_mode & 0o777)
        os.replace(name, dest)
    finally:
        Path(name).unlink(missing_ok=True)


def recover(root):
    work = root / '.install/update-transaction'
    journal = work / 'journal.json'
    if journal.exists():
        for entry in reversed(read_json(journal)):
            dest = destination(root, entry['name'])
            if entry['existed']:
                atomic_copy(work / 'backup' / entry['name'], dest)
            else:
                dest.unlink(missing_ok=True)
    if work.exists():
        shutil.rmtree(work)


def apply_files(root, stage, names):
    """Back up before changes; retain a journal until all replacements succeed."""
    recover(root)
    changed = []
    for name in names:
        source = stage / name
        dest = destination(root, name)
        if (dest.is_file() and dest.stat().st_size == source.stat().st_size
                and (dest.stat().st_mode & 0o777) == (source.stat().st_mode & 0o777)):
            with source.open('rb') as incoming, dest.open('rb') as existing:
                if hashlib.file_digest(incoming, 'sha256').digest() == hashlib.file_digest(existing, 'sha256').digest():
                    continue
        changed.append(name)
    names = changed
    if not names:
        return
    work = root / '.install/update-transaction'
    work.mkdir(parents=True)
    entries = []
    for name in names:
        dest = destination(root, name)
        entries.append({'name': name, 'existed': dest.exists()})
        if dest.exists():
            atomic_copy(dest, work / 'backup' / name)
    journal = work / 'journal.json'
    with journal.open('w', encoding='utf-8') as stream:
        json.dump(entries, stream)
        stream.flush()
        os.fsync(stream.fileno())
    try:
        for name in names:
            atomic_copy(stage / name, destination(root, name))
    except BaseException:
        recover(root)
        raise
    # Unlink is the commit point. A later startup cleans unused backups.
    journal.unlink()
    shutil.rmtree(work)


def validate_archive(archive, stage, expected, target):
    prefix = 'isaac_interactive_physics_lab/'
    with zipfile.ZipFile(archive) as z:
        infos = z.infolist()
        if len(infos) > 20000 or sum(i.file_size for i in infos) > 4 * 1024**3:
            raise ValueError('업데이트 압축 크기 제한 초과')
        names = [i.filename for i in infos if not i.is_dir()]
        if len(names) != len(set(names)):
            raise ValueError('중복된 업데이트 파일')
        manifest = json.loads(z.read(prefix + 'RELEASE_MANIFEST.json'))
        if manifest['target'] != target or manifest['build'] != expected['build']:
            raise ValueError('설치 대상 또는 버전 불일치')
        files = manifest['files']
        required = {'INSTALL_TARGET.json', 'run_isaac_with_browser.sh', 'tools/startup_update.py'}
        if not required.issubset(files):
            raise ValueError('필수 실행 파일 누락')
        if set(names) != {prefix + n for n in files} | {prefix + 'RELEASE_MANIFEST.json'}:
            raise ValueError('배포 파일 목록 불일치')
        for info in infos:
            if info.is_dir():
                continue
            name = info.filename[len(prefix):]
            safe_path(name)
            mode = info.external_attr >> 16
            if stat.S_ISLNK(mode):
                raise ValueError('업데이트에 심볼릭 링크가 있습니다.')
            data = z.read(info)
            if name != 'RELEASE_MANIFEST.json' and hashlib.sha256(data).hexdigest() != files[name]:
                raise ValueError('파일 검증 실패: ' + name)
            dest = stage / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(data)
            dest.chmod(0o755 if mode & 0o111 or name.endswith('.sh') else 0o644)
        if read_json(stage / 'INSTALL_TARGET.json') != target:
            raise ValueError('설치 대상 파일 불일치')
    return sorted(files) + ['RELEASE_MANIFEST.json']


def fetch(url, limit):
    request = urllib.request.Request(url, headers={'User-Agent': 'Isaac-AGV-Updater/1', 'Cache-Control': 'no-cache'})
    with urllib.request.urlopen(request, timeout=30) as response:
        data = response.read(limit + 1)
    if len(data) > limit:
        raise ValueError('응답 크기 제한 초과')
    return data


def update(root, report=lambda message: None):
    target = read_json(root / 'INSTALL_TARGET.json')
    if target.get('isaac') != '5.1' or target.get('architecture') != 'aarch64':
        raise ValueError('이 설치 대상의 자동 업데이트 배포 경로가 아직 없습니다.')
    report('최신 버전을 확인하고 있습니다…')
    latest = json.loads(fetch(BASE + ASSET + '.update.json', 16384))
    if latest.get('asset') != ASSET or latest.get('target') != target:
        raise ValueError('업데이트 배포 대상 불일치')
    if not re.fullmatch(r'\d{8}_\d{6}', latest.get('build', '')) or not re.fullmatch(r'[0-9a-f]{64}', latest.get('sha256', '')):
        raise ValueError('잘못된 배포 버전 정보')
    current = read_json(root / 'RELEASE_MANIFEST.json')['build']
    if latest['build'] <= current:
        return False
    size = latest.get('bytes')
    if not isinstance(size, int) or not 0 < size <= MAX_DOWNLOAD:
        raise ValueError('잘못된 다운로드 크기')
    # Staging and backups stay on the installation filesystem.
    with tempfile.TemporaryDirectory(prefix='.update-stage-', dir=root) as folder:
        work = Path(folder)
        archive = work / 'release.zip'
        report('새 버전 ' + latest['build'] + ' 다운로드 중…')
        req = urllib.request.Request(BASE + ASSET, headers={'User-Agent': 'Isaac-AGV-Updater/1', 'Cache-Control': 'no-cache'})
        digest = hashlib.sha256()
        received = 0
        with urllib.request.urlopen(req, timeout=60) as response, archive.open('wb') as output:
            while chunk := response.read(1024 * 1024):
                received += len(chunk)
                if received > size:
                    raise ValueError('다운로드 크기 불일치')
                digest.update(chunk)
                output.write(chunk)
                report(f'다운로드 중… {received * 100 // size}%')
        if received != size or digest.hexdigest() != latest['sha256']:
            raise ValueError('다운로드 검증 실패. 기존 버전을 유지합니다.')
        report('파일 검증 및 업데이트 적용 중…')
        stage = work / 'files'
        names = validate_archive(archive, stage, latest, target)
        apply_files(root, stage, names)
    return True


def start_dialog(args, **kwargs):
    from dialog_position import position_dialog
    env = os.environ.copy()
    if env.get('DISPLAY'):
        # GTK4 Zenity ignores --attach. Use X11 to set a real transient parent.
        env['GDK_BACKEND'] = 'x11'
    process = subprocess.Popen(args, env=env, **kwargs)
    position_dialog(process)
    return process


def dialog(message, primary, extras=()):
    if shutil.which('zenity') and (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        args = ['zenity', '--info', '--no-markup', '--title=AGV 실행', '--width=480',
                '--text=' + message, '--ok-label=' + primary]
        args += ['--extra-button=' + label for label in extras]
        process = start_dialog(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        output, _ = process.communicate()
        # Zenity versions disagree on the exit code for --extra-button. The
        # emitted label is authoritative, so do not turn "그냥 실행" into
        # "종료" merely because that version returned 1.
        choice = output.strip()
        if choice in (primary, *extras):
            return choice
        return primary if process.returncode == 0 else '종료'
    if sys.stdin.isatty():
        options = [primary, *extras]
        print(message)
        print(' / '.join(f'{i+1}: {v}' for i, v in enumerate(options)))
        answer = input('선택: ').strip()
        return options[int(answer)-1] if answer.isdigit() and 1 <= int(answer) <= len(options) else '종료'
    print('업데이트 안내 창을 열 수 없습니다. 데스크톱 터미널에서 실행하거나 zenity를 설치하세요.', file=sys.stderr)
    return '종료'


def main():
    from dialog_position import remember_terminal
    remember_terminal()
    # Recovery must succeed before launching, including after interrupted updates.
    try:
        recover(ROOT)
    except Exception as error:
        dialog('이전 업데이트 복구에 실패했습니다. 실행을 중단합니다.\n' + str(error), '종료')
        return 10
    if os.environ.get('LAB_SKIP_UPDATE_ONCE') == '1':
        return 0
    current = read_json(ROOT / 'RELEASE_MANIFEST.json').get('build', '알 수 없음')
    choice = dialog('현재 버전: ' + current + '\n\n실행 전에 업데이트를 확인하시겠습니까?\n새 버전이 있으면 업데이트한 뒤 실행합니다.',
                    '업데이트 확인', ('그냥 실행', '종료'))
    if choice != '업데이트 확인':
        return 0 if choice == '그냥 실행' else 10
    progress = None
    try:
        if shutil.which('zenity') and (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            progress = start_dialog(['zenity', '--progress', '--pulsate', '--auto-close', '--no-cancel',
                                         '--title=AGV 업데이트', '--text=최신 버전 확인 중…', '--width=480'],
                                        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, text=True)
        def report(message):
            print(message, flush=True)
            if progress and progress.poll() is None:
                try:
                    progress.stdin.write('# ' + message + '\n')
                    progress.stdin.flush()
                except BrokenPipeError:
                    pass
        changed = update(ROOT, report)
    except Exception as error:
        if progress and progress.poll() is None:
            progress.terminate()
            progress.wait()
        # If rollback itself failed, never offer to launch mixed files.
        if (ROOT / '.install/update-transaction/journal.json').exists():
            dialog('업데이트 복구가 필요하여 실행을 중단합니다.\n' + str(error), '종료')
            return 10
        choice = dialog('업데이트하지 못했습니다.\n' + str(error) + '\n\n기존 버전으로 실행할 수 있습니다.', '그냥 실행', ('종료',))
        return 0 if choice == '그냥 실행' else 10
    finally:
        if progress and progress.poll() is None:
            progress.terminate()
            progress.wait()
    print('업데이트를 완료했습니다. 새 버전으로 실행합니다.' if changed else '최신 버전입니다. 현재 버전으로 실행합니다.', flush=True)
    return 20 if changed else 0


if __name__ == '__main__':
    sys.exit(main())
