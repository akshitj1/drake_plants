# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

FROM ubuntu:bionic
ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# set -- errexit, no-unset, xtrace
RUN set -eux \
    && apt-get update \
    && apt-get -y install --no-install-recommends apt-utils dialog 2>&1 \
    # Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # [Optional] Add sudo support for non-root user
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# c++ image

# wrap multiple run into single only when seperated runs can cache-malfunction
# ref: https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#apt-get
# or
# if we want to avoid layers ie. each RUN, COPY, ADD creates new layer
# ref: https://dzone.com/articles/docker-layers-explained
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
    build-essential \
    cmake \
    # bug analysis
    cppcheck \
    # debug tools
    valgrind \
    # process tools
    procps \
    # useful for CLI installs
    lsb-release

# binary distribution image

WORKDIR /drake
COPY setup/packages-binary-req.txt setup/
RUN xargs -a setup/packages-binary-req.txt apt-get -y install --no-install-recommends \
    # Cleanup (todo: remove)
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# todo: reomve
# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=
ENV DEBIAN_FRONTEND=noninteractive

# source distribution image

RUN apt-get update \
    && apt-get -y install --no-install-recommends \
    apt-transport-https \
    ca-certificates \
    gnupg \
    wget

COPY setup/packages-source-req.txt setup/
RUN wget -O - https://drake-apt.csail.mit.edu/drake.pub.gpg | apt-key add \
    && echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/bionic bionic main" > /etc/apt/sources.list.d/drake.list \
    && apt-get update \
    && xargs -a setup/packages-source-req.txt apt-get -y install --no-install-recommends

RUN \
    locale-gen en_US.UTF-8 \
    && bazel_ver=1.1.0 \
    && bazel_url=https://releases.bazel.build/${bazel_ver}/release/bazel_${bazel_ver}-linux-x86_64.deb \
    # Download todo: verify
    && tmpdeb="/tmp/bazel_${bazel_ver}-amd64.deb" \
    && wget -O "${tmpdeb}" "${bazel_url}" \
    # Install.
    && dpkg -i "${tmpdeb}" \
    && rm "${tmpdeb}"
