from setuptools import setup

package_name = 'openai_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
)
