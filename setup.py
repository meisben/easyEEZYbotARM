# -*- coding: utf-8 -*-
"""Setup file for easyEEZYbotARM
"""

from setuptools import setup

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name="easyEEZYbotARM",
    version="0.0.1",
    description="A python controller (3 dimensions inverse and forward kinematics) for the EEZYbotARM (MK1,MK2,MK3) movement",
    license="MIT",
    long_description=long_description,
    author="Ben Money-Coomes",
    author_email="ben.money@gmail.com",
    url="https://github.com/meisben",
    package_dir={'': 'python_packages'},
    packages=["easyEEZYbotARM", ],
    install_requires=["scipy", "numpy", "matplotlib", "pySerial"]
)
