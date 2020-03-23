#!/bin/bash

# Davidnet (david@kiwibot.com)
# JohnBetaCode (john@kiwibot.com)
set -euo pipefail
IFS=$'\n\t'

# Download from nvidia packages to build GPU containers for balena
NVIDIA_URL="http://169.44.201.108:7002/jetpacks/4.3/"
FILE_LIST=(
    "Jetson_Linux_R32.2.3_aarch64.tbz2"
    "cuda-repo-l4t-10-0-local-10.0.326_1.0-1_arm64.deb"
    "libcudnn7_7.5.0.56-1+cuda10.0_arm64.deb" 
    "libcudnn7-dev_7.5.0.56-1+cuda10.0_arm64.deb"
    "libnvinfer5_5.1.6-1+cuda10.0_arm64.deb"
    "libnvinfer-dev_5.1.6-1+cuda10.0_arm64.deb"
)
URL_LIST=()
TEGRA_DRIVERS=${FILE_LIST[0]}
for i in ${FILE_LIST[@]}; do
    URL_LIST+=("$NVIDIA_URL$i")
done
printf "%s\n" "${URL_LIST[@]}"
echo "downloading files ..."

# Download in parallel all packages, if exits then continue
echo ${URL_LIST[@]} | sed 's/\*\*/ -P /g' | xargs -n 1 -P 8 wget -nc -q

# TODO(davidnet): Add to get untar to tmp files and get the all the packages from the Tegra186 folder
# Create temporal folder
TMPDIR=$(mktemp -d)
cp $TEGRA_DRIVERS $TMPDIR
pushd $TMPDIR
echo "temporal directory:" $TMPDIR

tar --strip-components=2 -xvf $TEGRA_DRIVERS --wildcards --no-anchored 'Linux_for_Tegra/nv_tegra/*.tbz2'
rm -rf $TEGRA_DRIVERS
mv nv_sample_apps/nvgstapps.tbz2 .
rm -rf nv_sample_apps
popd

mv $TMPDIR/* .

echo "L4T files downloaded"