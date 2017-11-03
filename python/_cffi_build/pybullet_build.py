from cffi import FFI
from os import path
import platform


HERE = path.dirname(path.realpath(__file__))


# ----------
# BUILD WRAPPER
# ----------
ffi = FFI()

# read file
with open(path.join(HERE, 'bullet.h')) as f:
    raw_header = f.read()

# prepare cdef and source
cdef = raw_header

source = raw_header

# configure cffi
ffi.cdef(cdef)
ffi.set_source('_pybullet', source)


if __name__ == '__main__':
    ffi.compile(verbose=True)
