from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

assets = {}
for file in Path('web/dist').rglob('*'):
    if file.is_file():
        assets.setdefault('share/mower_console/' + str(file.parent), []).append(str(file))
setup(name='mower_console', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/mower_console']),
                  ('share/mower_console', ['package.xml', 'README.md']),
                  ('share/mower_console/config', glob('config/*')),
                  ('share/mower_console/launch', glob('launch/*')),
                  ('share/mower_console/systemd', glob('systemd/*')),
                  ('share/mower_console/scripts', glob('scripts/*'))] + list(assets.items()),
      install_requires=['setuptools', 'aiohttp', 'PyYAML', 'numpy'],
      entry_points={'console_scripts': ['console_server = mower_console.server:main']})
