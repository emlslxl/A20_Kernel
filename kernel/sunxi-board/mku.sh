#!/bin/sh

#./script $1
outname=$1
outname=${outname%%.*}.bin
./fex2bin $1 ${outname}
mkimage -A arm -C none -a 0x43000000 -n script -d ${outname} uscript.bin
