from distutils.core import setup, Extension

setup(
    name="globe",
    author="Stephen M. Herwig",
    author_email="smherwig@cs.umd.edu",
    version="0.1",
    description="utilities to calculate distances and angles on the Earth globe",
    ext_modules=[Extension("globe", ["globe.c", "globemodule.c"])]
    )
