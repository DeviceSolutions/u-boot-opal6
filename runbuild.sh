#!/bin/bash

RELEASE_DIR=releases

function buildit
{
    MACHINE=$1

    if [ -z $2 ]
    then
        MACHINE_DIR=$1
    else
        MACHINE_DIR=$2
    fi

    OUTPUT_DIR=$RELEASE_DIR/$MACHINE_DIR
    mkdir -p $OUTPUT_DIR
    rm -f $OUTPUT_DIR/*.imx

    CONF=${MACHINE}_mfgtool_defconfig

    echo "building u-boot for $1 mfgtool"

    echo 'cleaning...'
    make clean

    echo 'configuring...'
    make $CONF

    echo 'compiling...'
    make

    mv u-boot.imx $OUTPUT_DIR/u-boot-mfgtool.imx

    CONF=${1}_defconfig

    echo "building u-boot for $1 mfgtool"

    echo 'cleaning...'
    make clean

    echo 'configuring...'
    make $CONF

    echo 'compiling...'
    make

    mv u-boot.imx $OUTPUT_DIR/u-boot.imx
}

if [ -z $1 ]
then
    echo 'building for all machines'
    buildit opal6dl1g DualLite1GB
    buildit opal6dl2g DualLite2GB
    buildit opal6q1g Quad1GB
    buildit opal6q2g Quad2GB
    buildit opal6s1g Solo1GB
else
    echo "building for $1"
    buildit $1 $2
fi

echo "Done. U-boot images at $RELEASE_DIR"