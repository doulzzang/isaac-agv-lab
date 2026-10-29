#!/usr/bin/env python3
"""Prepare local browser/Spark and homepage ZIPs together; never publish or push."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
SPARK_NAME='isaac_interactive_physics_lab_spark_isaac51_20260913_154402.zip'


def homepage_sources(homepage):
    """Use Git when available, or a verified release snapshot without inventing a commit."""
    probe=subprocess.run(['git','rev-parse','--show-toplevel'],cwd=homepage,
                         capture_output=True,text=True)
    if probe.returncode == 0 and Path(probe.stdout.strip()).resolve() == homepage.resolve():
        names=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=homepage).decode().split('\0')
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=homepage,text=True).strip()
        return sorted(set(n for n in names if n)), commit, 'Git working tree'
    snapshot=json.loads((homepage/'SNAPSHOT_MANIFEST.json').read_text())
    names=sorted(snapshot['files'])
    for name in names:
        relative=Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Unsafe homepage snapshot path: '+name)
        path=homepage/relative
        if not path.is_file() or path.is_symlink():
            raise ValueError('Missing homepage snapshot file: '+name)
        with path.open('rb') as stream:
            digest=hashlib.file_digest(stream,'sha256').hexdigest()
        if digest != snapshot['files'][name]:
            raise ValueError('Homepage snapshot hash mismatch: '+name)
    if not names:
        raise ValueError('Empty homepage snapshot')
    return names, snapshot['base_commit'], 'Verified archived snapshot; homepage unchanged'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--homepage-root',type=Path,required=True)
    parser.add_argument('--validation-report',type=Path,required=True)
    parser.add_argument('--source-revision',default='2026-10-29-code-update')
    args=parser.parse_args()
    homepage=args.homepage_root.resolve()
    report=args.validation_report.resolve()
    if not report.is_file():parser.error('검증 보고서가 필요합니다.')
    names,base_commit,source=homepage_sources(homepage)
    stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    destination=ROOT/'release';destination.mkdir(exist_ok=True)
    subprocess.run([sys.executable,str(ROOT/'tools/build_release.py'),'--target','spark_isaac51',
                    '--output-name',SPARK_NAME,'--source-revision',args.source_revision,
                    '--validation-scope',report.read_text()],check=True)
    output=destination/f'doulzzang_homepage_{stamp}.zip'
    manifest=dict(base_commit=base_commit,source=source,published=False,files={})
    with zipfile.ZipFile(output,'x',zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for name in names:
            path=homepage/name
            if not path.is_file() or path.is_symlink():continue
            if any(part in {'.git','node_modules','.wrangler','.next','dist'} for part in path.relative_to(homepage).parts):continue
            if path.name.startswith('.env') and not path.name.endswith(('.example','.sample')):continue
            data=path.read_bytes();manifest['files'][name]=hashlib.sha256(data).hexdigest()
            archive.writestr('doulzzang_homepage/'+name,data)
        archive.writestr('doulzzang_homepage/SNAPSHOT_MANIFEST.json',json.dumps(manifest,ensure_ascii=False,indent=2))
        archive.writestr('doulzzang_homepage/VALIDATION_REPORT.md',report.read_text())
    with zipfile.ZipFile(output) as archive:
        if archive.testzip():raise RuntimeError('Homepage ZIP CRC failure')
    digest=hashlib.file_digest(output.open('rb'),'sha256').hexdigest()
    output.with_suffix('.zip.sha256').write_text(f'{digest}  {output.name}\n')
    pair=dict(prepared_at=stamp,publication_requires_user_approval=True,published=False,
              browser_zip=SPARK_NAME,homepage_zip=output.name,validation_report=str(report),
              homepage_sha256=digest,homepage_source=source,
              source_revision=args.source_revision,
              browser_sha256=(destination/(SPARK_NAME+'.sha256')).read_text().split()[0])
    (destination/'LATEST_RELEASE_PAIR.json').write_text(json.dumps(pair,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(pair,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
