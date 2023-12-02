FROM ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive

# Install some necessary packages
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  ccache \
  bear \
  clangd \
  git \
  doxygen \
  graphviz \
  gdb \
  cpplint \
  valgrind \
  wget \
  libeigen3-dev \
  libcppunit-dev \
  python3-pip \
  python3-numpy \
  python3-matplotlib

# Add a non-root user
ARG USERNAME=terraformer-dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

ENV DEBIAN_FRONTEND=noninteractive
