"""Fetch the pinned AWS world once; retain upstream licensing and model files."""
import argparse
import hashlib
from pathlib import Path
import shutil
import tarfile
import tempfile
import urllib.request

from .model import load_config

REVISION = 'ff9631ca6d1db9c1ba656498151464b5ab74aafe'
SHA256 = 'e459bd9d7bdabdfc40f8afc6770ceb1d774316e5da94a1048f036baa7388b2d9'
URL = f'https://codeload.github.com/aws-robotics/aws-robomaker-small-house-world/tar.gz/{REVISION}'


def repair_photo_paths(destination):
    """Correct upstream photo paths which traverse one directory too many."""
    for mesh in (destination / 'models').glob('*/meshes/*.DAE'):
        content = mesh.read_bytes()
        repaired = content.replace(b'../../../../photos/', b'../../../photos/')
        if repaired != content:
            mesh.write_bytes(repaired)


def ensure_assets(config, archive=None):
    cfg = load_config(config)
    if cfg['gazebo']['world_file']:
        return
    destination = Path(cfg['gazebo']['aws_house_dir'])
    marker = destination / '.upstream_revision'
    if marker.is_file() and marker.read_text().strip() == REVISION:
        if (destination / 'worlds/small_house.world').is_file() and (destination / 'models').is_dir():
            repair_photo_paths(destination)
            print(f'AWS Small House ready: {destination}')
            return
    if destination.exists():
        raise RuntimeError(f'{destination} exists but is incomplete or has another revision; choose a new aws_house_dir')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='aws-house-', dir=destination.parent) as temporary:
        staging = Path(temporary)
        if archive is None:
            archive = staging / 'world.tar.gz'
            print(f'Downloading AWS Small House {REVISION} (about 82 MiB)', flush=True)
            with urllib.request.urlopen(URL, timeout=120) as response, archive.open('wb') as output:
                shutil.copyfileobj(response, output)
        archive = Path(archive)
        with archive.open('rb') as stream:
            actual = hashlib.file_digest(stream, 'sha256').hexdigest()
        if actual != SHA256:
            raise RuntimeError(f'AWS archive checksum mismatch: {actual}')
        with tarfile.open(archive) as bundle:
            bundle.extractall(staging / 'unpacked', filter='data')
        extracted = staging / 'unpacked' / f'aws-robomaker-small-house-world-{REVISION}'
        (extracted / '.upstream_revision').write_text(REVISION + '\n')
        # Prevent accidental colcon discovery of the legacy gazebo_ros package.
        (extracted / 'COLCON_IGNORE').touch()
        repair_photo_paths(extracted)
        extracted.rename(destination)
    print(f'AWS Small House installed: {destination}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('config')
    parser.add_argument('--archive', type=Path)
    args = parser.parse_args()
    ensure_assets(args.config, args.archive)
