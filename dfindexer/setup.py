import sys
from setuptools import setup, Extension
from Cython.Build import cythonize

debug_build = "--debug" in sys.argv
if debug_build:
    sys.argv.remove("--debug")

extra_compile_args = ["-g", "-O0"] if debug_build else []
extra_link_args = ["-g"] if debug_build else []

ext = Extension(
    name="dfindexer",
    sources=["dfindexer_cy.pyx", "dfindexer.c"],
    include_dirs=["."],
    extra_compile_args=extra_compile_args,
    extra_link_args=extra_link_args,
)

setup(
    ext_modules=cythonize([ext], language_level="3"),
)
