#!/bin/sh
# make an abin file for a firmware this file format is for sending to
# a memory constrained companion computer to flash over serial to a
# flight board

if [ $# -lt 1 ]; then
    # default to FMUv3
    ELF=build/px4-v3/bin/arducopter
else
    ELF=$1
fi

[ -f $ELF ] || {
    echo "Can't find ELF file"
    exit 1
}

echo "Creating arducopter.bin"
arm-none-eabi-objcopy -O binary "$ELF" arducopter.bin || {
    echo "Failed to create bin file"
    exit 1
}

sum=$(md5sum arducopter.bin | cut -d' ' -f1)
githash=$(git rev-parse HEAD)

echo "githash $githash md5 $sum"

cat <<EOF > arducopter.abin
git version: $githash
MD5: $sum
--
EOF
cat arducopter.bin >> arducopter.abin

echo "Created arducopter.abin"
