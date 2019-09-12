# CMake Project with an Installed Drake

This uses the CMake `find_package(drake)` mechanism to find an installed instance of Drake.

# Instructions

These instructions are only supported for Ubuntu 16.04 (Xenial).

```shell
###############################################################
# Install Prerequisites
###############################################################
install gflags following stesp:
`https://github.com/gflags/gflags/blob/master/INSTALL.md`

###############################################################
# Install Drake to /opt/drake
###############################################################

# 1) A specific version (date-stamped)
# curl -O https://drake-packages.csail.mit.edu/drake/nightly/drake-20171015-xenial.tar.gz

# 2) The latest (usually last night's build)
curl -O https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-xenial.tar.gz
sudo tar -xvzf drake-latest-xenial.tar.gz -C /opt

###############################################################
# Build Everything
###############################################################
mkdir build && cd build
cmake -DCMAKE_PREFIX_PATH=/opt/drake ..
make

###############################################################
# Execute
###############################################################
# A demo
cd src/glider && exec ./simulate

# (Optionally) Run Tests
make test
```
