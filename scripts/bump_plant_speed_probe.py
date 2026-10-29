#!/usr/bin/env python3
"""Compare two speed targets using the published MecoSimulation bump plant test."""
import argparse
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path


def run(cmd, **kwargs):
    return subprocess.run(cmd, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-repo', required=True, type=Path)
    parser.add_argument('--archive', required=True, type=Path,
                        help='published 2026.0.14 desktop static adapter archive')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    git = ['git', '-C', str(args.sim_repo)]
    revision = subprocess.check_output(git + ['rev-parse', 'v2026.0.14^{commit}'], text=True).strip()
    source = subprocess.check_output(git + ['show', 'v2026.0.14:tests/BumpTraversalTest.cpp'], text=True)
    edits = [
        ('for (int scenario : {0, 2})', 'for (int scenario : {2})'),
        ('const double target = 5.24;', 'const double target = TARGET_SPEED;'),
        ('if (scenario == 2 && (!cleared || maxTilt > pi / 4))',
         'if (false && scenario == 2 && (!cleared || maxTilt > pi / 4))'),
        ('c.rideHeight = .0762;', 'c.rideHeight = .08255;'),
        ('c.centerOfMass.z = .2238;', 'c.centerOfMass.z = .21745;'),
        ('RobotContactConfig contact;', 'RobotContactConfig contact; contact.dampingRatio = 1.2;'),
    ]
    for old, new in edits:
        if source.count(old) != 1:
            raise ValueError(f'published test no longer matches expected text: {old}')
        source = source.replace(old, new)
    archive = args.archive.resolve()
    with tempfile.TemporaryDirectory(prefix='meco-bump-probe-') as tmp:
        root = Path(tmp)
        headers = subprocess.check_output(git + ['archive', 'v2026.0.14', 'include'])
        run(['tar', '-x', '-C', str(root)], input=headers)
        (root / 'probe.cpp').write_text(source)
        results = {}
        for speed in ('1.0', '5.24'):
            exe = root / 'probe'
            run(['g++', '-std=c++23', '-O2', '-DTARGET_SPEED='+speed,
                 '-I'+str(root / 'include'), str(root / 'probe.cpp'), str(archive),
                 '-pthread', '-o', str(exe)])
            results[speed] = subprocess.check_output([str(exe)], text=True).strip()
    result = {
        'scope': 'Published numerical plant test, simplified speed controller, not the robot scheduler or hardware.',
        'revision': revision,
        'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
        'adapted_source_sha256': hashlib.sha256(source.encode()).hexdigest(),
        'changes': 'Compliant case only; compare targets; report non-clearance; use robot ride height, center of mass and foam damping.',
        'step_seconds': .004,
        'maximum_steps': 2500,
        'results': results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
