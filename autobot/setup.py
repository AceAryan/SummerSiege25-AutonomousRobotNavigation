from setuptools import setup

package_name = 'autobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ace',
    maintainer_email='ace@example.com',
    description='Autobot package with APF navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apf_navigator = apf_navigator:main',
        ],
    },
)
