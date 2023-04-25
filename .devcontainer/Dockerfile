FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# TODO check for cmake cppcheck valgrind lldb llvm gdb
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
    apt-utils \    
    build-essential \
    ca-certificates \
    curl \
    dialog \
    git \
    screen \
    udev \
    unzip \
    vim \
    wget \
    # for platformio
    bc \
    build-essential \
    clang \
    curl \
    gcc \
    libpython2.7 \
    libusb-1.0-0 \
    python3 \
    python3-dev \
    python3-distutils \
    python3-pip \
    python3-venv \
    srecord \
    xz-utils \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Do not run as root, use sudo without password; might be useless
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

# https://docs.platformio.org/en/latest/faq.html#platformio-udev-rules
RUN curl -fLo /etc/udev/rules.d/99-platformio-udev.rules --create-dirs \
    https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
    && usermod -a -G dialout $USERNAME \
    && usermod -a -G plugdev $USERNAME

USER $USERNAME

# install PlatformIO
RUN python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/develop/scripts/get-platformio.py)"

# simulator dependencies
RUN python3 -m pip install pyserial