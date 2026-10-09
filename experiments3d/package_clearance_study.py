"""Copy reviewable evidence; optionally archive all numerical data on F drive.

Run only after numerical and postprocessing completion. Original data are kept.
Git history is exported separately with `git bundle create` after the final commit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import time
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def save_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')


def evidence_files(study: Path):
    for path in sorted(study.iterdir()):
        if path.is_file() and path.suffix.lower() in {'.csv', '.json', '.md', '.html'}:
            yield path
    for folder in ['figures', 'examples', 'validation']:
        if (study / folder).exists():
            yield from sorted(path for path in (study / folder).rglob('*') if path.is_file())


def copy_evidence(study: Path, destination: Path) -> int:
    destination.mkdir(parents=True, exist_ok=True)
    count = 0
    for source in evidence_files(study):
        target = destination / source.relative_to(study)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        count += 1
    return count


def package(study: Path, delivery: Path, archive: bool, bundle: Path | None):
    assert study.is_relative_to(ROOT / 'results'), 'Expected the original F-drive study'
    assert delivery.is_relative_to(ROOT / '交付文件'), 'Delivery must remain on F drive'
    assert json.loads((study / 'execution_complete.json').read_text())
    assert json.loads((study / 'postprocessing_complete.json').read_text())['passed']
    audit = json.loads((study / 'raw_mat_audit.json').read_text())
    assert audit['passed'] and audit['executionErrors'] == 0
    assert audit['records'] == audit['checked'] == 8422 and audit['mapGeometryCount'] == 2045
    integrity = json.loads((study / 'file_integrity.json').read_text())
    assert integrity['sourceHashesMatch'] and integrity['rawMatCount'] == 8422
    assert json.loads((study / 'replay_validation.json').read_text())['passed']
    assert (study / 'examples/staggered_windows/flight_3d.mp4').is_file()
    assert json.loads((study / 'graphics_source_sha256.json').read_text())['passed']
    assert len(list((study / 'figures').glob('*_data.csv'))) == 8
    for extension in ['png', 'pdf', 'fig']:
        assert len(list((study / 'figures').glob('*.' + extension))) == 8
    analysis = json.loads((study / 'analysis.json').read_text())
    assert analysis['scheduled'] == 8422
    count = copy_evidence(study, delivery)
    tracked = ROOT / 'docs/three_dimensional/clearance_study_20261006'
    # Keep the short demonstration movie beside the report in Git so its
    # relative links work. Large raw MATs stay in the verified data archive.
    tracked.mkdir(parents=True, exist_ok=True)
    for source in evidence_files(study):
        if source.suffix.lower() == '.mat':
            continue
        target = tracked / source.relative_to(study)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    tools = delivery / 'code_tools'
    tools.mkdir(exist_ok=True)
    for relative in [
        'config/clearance_candidate_params3d.m',
        'experiments3d/replay_clearance_seed3d.m',
        'experiments3d/validate_clearance_replay3d.m',
        'experiments3d/execute_clearance_replay_validation.py',
        'experiments3d/README_CLEARANCE_STUDY.md',
        'experiments3d/package_clearance_study.py',
    ]:
        shutil.copy2(ROOT / relative, tools / Path(relative).name)
    receipt = dict(
        study=str(study), delivery=str(delivery), copiedEvidenceFiles=count,
        sourceCommit=analysis['sourceCommit'], scheduled=analysis['scheduled'],
        selectedConfig=analysis['final'], rawAuditPassed=True,
        baselineTag='baseline-3d-v2.1.0-20261006',
        numericalTag='experiment-clearance-run-v2.2.0-20261006',
        completedTag='clearance-study-v2.2.0-20261006',
        originalDataPreserved=True, createdLocal=time.strftime('%Y-%m-%d %H:%M:%S'),
    )
    if archive:
        old = ROOT / 'results/seed_sweep3d_v2.1.0_20261005_113155_970'
        target = delivery / 'experiment_data_full.zip'
        assert not target.exists(), 'Preserve an existing archive; use a new delivery directory'
        assert old.is_dir()
        total = 0
        with zipfile.ZipFile(target, 'w', compression=zipfile.ZIP_STORED, allowZip64=True) as output:
            for source_root in [study, old]:
                for source in sorted(source_root.rglob('*')):
                    if not source.is_file() or '__pycache__' in source.parts:
                        continue
                    assert source.resolve().is_relative_to(source_root.resolve())
                    output.write(source, str(Path(source_root.name) / source.relative_to(source_root)))
                    total += 1
                    if total % 1000 == 0:
                        print(f'ARCHIVE_PROGRESS files={total}', flush=True)
        with zipfile.ZipFile(target) as check:
            assert check.testzip() is None, 'ZIP CRC check failed'
            archived_count = len(check.infolist())
        receipt['archive'] = dict(path=str(target), bytes=target.stat().st_size,
                                  files=archived_count, sha256=digest(target), crcPassed=True,
                                  includes=[study.name, old.name])
    if bundle:
        assert bundle.is_file() and bundle.resolve().is_relative_to(delivery)
        receipt['gitBundle'] = dict(path=str(bundle), bytes=bundle.stat().st_size, sha256=digest(bundle))
    files = [path for path in delivery.rglob('*') if path.is_file()
             and path.name not in {'delivery_receipt.json', 'delivery_sha256.csv', 'experiment_data_full.zip'}]
    import csv
    with (delivery / 'delivery_sha256.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['relativePath', 'bytes', 'sha256'])
        for path in sorted(files):
            writer.writerow([path.relative_to(delivery).as_posix(), path.stat().st_size, digest(path)])
    save_json(delivery / 'delivery_receipt.json', receipt)
    print('EVIDENCE_PACKAGED ' + str(delivery), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('study', type=Path)
    parser.add_argument('delivery', type=Path)
    parser.add_argument('--archive', action='store_true')
    parser.add_argument('--bundle', type=Path)
    args = parser.parse_args()
    package(args.study.resolve(), args.delivery.resolve(), args.archive,
            args.bundle.resolve() if args.bundle else None)
