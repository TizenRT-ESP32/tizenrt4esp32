#!/bin/bash
###########################################################################
#
# Copyright 2018 Samsung Electronics All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific
# language governing permissions and limitations under the License.
#
###########################################################################
#
# File   : artik05x_ota.sh
# Description : Attach CRC and Signing in TizenRT Binary

source $(dirname "${BASH_SOURCE[0]}")/artik05x_cmn.sh
TARGET_NAME=ota
OUTPUT_BIN=$OUTPUT_BINARY_PATH/${TARGET_NAME}.bin

make-target-bin() {
    local bin=
    local obj=
    local optarg=

    while test $# -gt 0; do
        optarg=`echo "$1" | sed 's/[-_a-zA-Z0-9]*=//'`

        case $1 in
            --target=*) t=$optarg ;;
            --bin=*)    bin=$OUTPUT_BINARY_PATH/$optarg.bin ;;
        esac
        shift
    done

    case $t in
        factory)
            gzip -c $TIZENRT_BIN > $bin
            ;;
        ota)
            obj=`make-target-bin --target=factory --bin=tmp` || return 1
            dd if=$obj of=$bin bs=1024 seek=4;
            echo $(printf "%08x" $(cat $obj | wc -c)) | tac -rs .. | xxd -r -p | \
                dd of=$bin bs=1 seek=0 conv=notrunc
            echo $(crc32 $obj) | tac -rs .. | xxd -r -p | \
                dd of=$bin bs=1 seek=4 conv=notrunc
            rm -rf $obj
            ;;
        *)
            test ! -e $bin && return 1 ;;
    esac

    echo $bin

    return 0
}

ota_crc() {
    make-target-bin --target=$TARGET_NAME --bin=$TARGET_NAME
}

if test $# -eq 0; then
    exit 1
fi

while test $# -gt 0; do
    case "$1" in
        -*=*) optarg=`echo "$1" | sed 's/[-_a-zA-Z0-9]*=//'` ;;
        *) optarg= ;;
    esac

    case $1 in
        --board=*)
            BOARD_NAME=$optarg
            BOARD_DIR_PATH=${BUILD_DIR_PATH}/configs/$BOARD_NAME
            FW_DIR_PATH=${BOARD_DIR_PATH}/bin
            if [ ! -d $BOARD_DIR_PATH ]; then
                exit 1
            fi
            ;;
        --secure)
            signing
            ;;
        *)
            ;;
    esac
    shift
done
ota_crc
