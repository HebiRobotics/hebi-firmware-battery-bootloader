#!/bin/bash

# A script to upload all firmware to the firmware server

if [ $# != 4 ]
then
  echo "Usage: ./upload_all_firmware.sh <FW type (BETA/STABLE/etc)> <upload label (15.2.1, etc)> <upload token> <url>"
  exit 1
fi

TYPE=$1
LABEL=$2
TOKEN=$3
URL=$4
lowercase_type=$(echo ${TYPE} | tr '[:upper:]' '[:lower:]')

for VER in 'B'
do
  ver=$(echo ${VER} | tr '[:upper:]' '[:lower:]')
  ./common/scripts/upload_single_firmware.sh \
    ${TYPE} \
    BIB_BOOT_${VER} \
    bib_boot_${ver}.bin \
    bib_${ver}-${lowercase_type}.bin \
    ${LABEL} \
    ${TOKEN} \
    ${URL}
done