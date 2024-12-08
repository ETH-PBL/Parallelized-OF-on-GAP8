FROM ubuntu:focal
ARG USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Ensure necessary directories exist
RUN mkdir -p /home/$USERNAME/ws/src \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ws/
RUN apt-get update && apt-get upgrade -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y python3-pip git rsync language-pack-en udev cmake
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

COPY .bashrc_git /home/$USERNAME/
RUN echo "source ~/.bashrc_git" >> /home/$USERNAME/.bashrc
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10

USER $USERNAME
SHELL ["/bin/bash", "-c"]
RUN cd /home/$USERNAME/ws && \
    git clone https://github.com/GreenWaves-Technologies/gap_riscv_toolchain_ubuntu.git && \
    cd /home/$USERNAME/ws/gap_riscv_toolchain_ubuntu && \
    ./install.sh /usr/lib/gap_riscv_toolchain

RUN cd /home/$USERNAME/ws && \
    git clone https://github.com/GreenWaves-Technologies/gap_sdk.git && \
    cd /home/$USERNAME/ws/gap_sdk && \
    cat requirements_apt.md | xargs sudo DEBIAN_FRONTEND=noninteractive apt install -y && \
    pip3 install -r requirements.txt && \
    pip3 install -r doc/requirements.txt

# add sourcing
RUN echo "alias GAP_SDK='source /home/$USERNAME/ws/gap_sdk/sourceme.sh'" >> /home/$USERNAME/.bashrc
CMD ["/bin/bash"]
