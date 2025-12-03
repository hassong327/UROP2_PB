from setuptools import setup, find_packages

setup(
    name="bezier_core",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "matplotlib",
        "scipy",
    ],
    author="",
    description="Shared Bezier path-planning core (geometry + QP push-away)",
    license="",
)
