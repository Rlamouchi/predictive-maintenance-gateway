# Yocto image

Custom Linux image for the gateway, built with Yocto (Poky scarthgap 5.0.21).
Current target: qemux86-64 (runs in QEMU). Raspberry Pi target planned later.

## Build

    mkdir -p ~/yocto && cd ~/yocto
    git clone --depth 1 -b scarthgap https://git.yoctoproject.org/poky
    cd poky && source oe-init-build-env ~/yocto/build
    cp <repo>/yocto/conf/local.conf <repo>/yocto/conf/bblayers.conf conf/
    bitbake core-image-minimal --runall=fetch    # 1. download all sources
    bitbake core-image-minimal                   # 2. compile (several hours the first time)

## Run

    runqemu qemux86-64 nographic slirp    # login: root (no password)
    poweroff                              # inside the image, to quit

## Notes

- Ubuntu 24.04 host: kernel.apparmor_restrict_unprivileged_userns = 0 (in /etc/sysctl.d/).
- INHERIT += "rm_work" keeps disk use low.
- Download step: BB_NUMBER_THREADS = "8". Compile step: "2" with PARALLEL_MAKE = "-j 2" (8 GB RAM VM).
- own-mirrors and a shallow clone of linux-yocto make the downloads much faster.
- Result: root filesystem of about 31 MB.
