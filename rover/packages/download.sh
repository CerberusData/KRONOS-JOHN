#!/bin/bash
#
# Download from nvidia packages to build GPU containers for balena
# Davidnet (david@kiwicampus.com)
set -euo pipefail
IFS=$'\n\t'

NVIDIA_URL="https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/4.2.3/lw.xd42/JetPackL4T_33_b39/"
# [1] https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/JETPACK_423_b6/
# [2] https://developer.nvidia.com/assets/embedded/secure/tools/files/jetpack-sdks/jetpack-4.2.1-ga/JETPACK_421_b97/P3448-0000/

FILE_LIST=(
    "Tegra186_Linux_R28.2.1_aarch64.tbz2"           # [OK][1] P3310/Jetson_Linux_R32.2.3_aarch64.tbz2
    "cuda-repo-l4t-9-0-local_9.0.252-1_arm64.deb"   # [OK][1] cuda-repo-l4t-10-0-local-10.0.326_1.0-1_arm64.deb
    "libcudnn7_7.1.5.14-1+cuda9.0_arm64.deb"        # [OK][2] libcudnn7_7.5.0.56-1+cuda10.0_arm64.deb
    "libcudnn7-dev_7.1.5.14-1+cuda9.0_arm64.deb"    # [OK][2] libcudnn7-dev_7.5.0.56-1+cuda10.0_arm64.deb
    "libnvinfer4_4.1.3-1+cuda9.0_arm64.deb"         # [OK][2] libnvinfer5_5.1.6-1+cuda10.0_arm64.deb
    "libnvinfer-dev_4.1.3-1+cuda9.0_arm64.deb"      # [OK][2] libnvinfer-dev_5.1.6-1+cuda10.0_arm64.deb
)

if [ -f "Tegra186_Linux_R28.2.1_aarch64.tbz2" ]; then
    echo "L4T files do exist in specified folder! Exiting..."
    exit 0
fi;

# TODO (davidnet): do this in parallel and wait for script to finish
for file in ${FILE_LIST[@]}; do
    wget --quiet --no-clobber "$NVIDIA_URL$file"
done

# TODO (davidnet): Add to get untar to tmp files and get the all the packages from the Tegra186 folder.
TMPDIR=$(mktemp -d) 

cp Tegra186_Linux_R28.2.1_aarch64.tbz2 $TMPDIR

pushd $TMPDIR
tar --strip-components=2 -xvf Tegra186_Linux_R28.2.1_aarch64.tbz2 --wildcards --no-anchored 'Linux_for_Tegra/nv_tegra/*.tbz2'
rm -rf Tegra186_Linux_R28.2.1_aarch64.tbz2
mv nv_sample_apps/nvgstapps.tbz2 .
rm -rf nv_sample_apps
popd

mv $TMPDIR/* .

echo "L4T files downloaded."