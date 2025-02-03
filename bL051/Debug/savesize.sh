#!/bin/sh

size=$(/usr/bin/size bL051.elf)
echo $size > ../size.txt
echo ''
size=$(stat --format '%s %N' bL051.bin)
echo $size >> ../size.txt

