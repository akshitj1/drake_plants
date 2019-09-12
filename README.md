# Flying plants and trajectory optimisation with installed instance of Drake

We may want to do this to use SNOPT being shipped with drake binary

This uses the CMake `find_package(drake)` mechanism to find an installed instance of Drake.
# Plants implemented
Perching Glider with Direct Collocation: ["Robust Post-Stall Perching with a Simple Fixed-Wing Glider using LQR-Trees"](https://groups.csail.mit.edu/robotics-center/public_papers/Moore14a.pdf) - J.Moore et. al.

## TODO:
..* LQR tree for Perching Glider
..* 2D tailsitter plant

# Instructions

These instructions are tested on unix.

```shell
###############################################################
# Install Prerequisites
###############################################################
install gflags following steps in:
`https://github.com/gflags/gflags/blob/master/INSTALL.md`

###############################################################
# Install Drake to /opt/drake
###############################################################

# 1) A specific version (date-stamped)
# curl -O https://drake-packages.csail.mit.edu/drake/nightly/drake-20171015-xenial.tar.gz

# 2) The latest (usually last night's build)
curl -O https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac.tar.gz
sudo tar -xvzf drake-latest-mac.tar.gz -C /opt

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
cd src/glider && exec ./simulate --spdlog_level trace

# (Optionally) Run Tests
make test
```
