import os
from setuptools import setup, find_packages

here = os.path.abspath(os.path.dirname(__file__))

try:
    with open(os.path.join(here, 'README.md'), 'r', encoding='utf-8') as fh:
        long_description = fh.read()
except (FileNotFoundError, IOError):
    long_description = "WidowX robot environments package"

setup(
    name='widowx_envs',
    version='0.0.1',
    packages=find_packages(),
    license='MIT License',
    long_description=long_description,
    long_description_content_type="text/markdown",
    entry_points={
        'console_scripts': [
            'widowx_env_service = widowx_envs.widowx_env_service:main',
        ],
    },
)
