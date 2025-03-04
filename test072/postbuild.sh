#!/bin/bash

f_name=test072
date > log

TOOLPATH=/opt/st/stm32cubeide_1.17.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.linux64_1.1.0.202410170702/tools/bin

absPtr=0
elfPtr=0
hexValue=0
searchElfSections()
{
    symbol=$1
    fullLine=$(readelf -S $f_name.elf | grep "$symbol")
    line=${fullLine#*\]}
    array=(  $line )
    absPtr=$(( 0x${array[2]} ))
    elfPtr=$(( 0x${array[3]} ))
}

echoHex()
{
     echo $(printf "%08x" $1) >> log 
}

#$TOOLPATH/arm-none-eabi-objcopy -O binary ${f_name}.elf  ${f_name}.bin

### calculate the crc


### Get section name properties from elf file

searchElfSections isr_vector
isrVectorElfPtr=$elfPtr
isrVectorAbsAddr=$absPtr
echo  isr_vector Elf and Abs: >> log
echo $(echoHex $isrVectorElfPtr) $(echoHex $isrVectorAbsAddr) >> log

searchElfSections crcPtr
crcElfPtr=$elfPtr
crcAbsPtr=$absPtr
echo crcPtr: >> log
echo $(echoHex $crcElfPtr) $(echoHex $crcAbsPtr) >> log 

searchElfSections crcLocation
crcLocationElfPtr=$elfPtr
crcLocationAbsPtr=$absPtr
echo crcLocation: >> log
echo $(echoHex $crcLocationElfPtr) $(echoHex $crcLocationAbsPtr) >> log 

#poke  the crc pointer value into the elf file 
crcElfPtrHex=$(printf '%x' $crcElfPtr)
crcLocationAbsPtrHex=$(printf '%08x'  $crcLocationAbsPtr)
echo  $crcElfPtrHex: $crcLocationAbsPtrHex > /tmp/crc
xxd -r /tmp/crc ${f_name}.elf

#generate a new bin file from elf
$TOOLPATH/arm-none-eabi-objcopy -O binary ${f_name}.elf  ${f_name}_crc.bin

###
# remove the crc placeholder
truncate --size=-4 ${f_name}_crc.bin
#fileLen=$(stat --printf="%s" ${f_name}_crc.bin) 
#crcLocationAbsAddr=$filelen
# get crc of this file
crcStr=$(crc32 ${f_name}_crc.bin)
crc=$(( 0x$crcStr ))
crcHex=$(printf "%08x" $crc)
echo crc: $crcStr $crc $crcHex >> log

# patch the crc value to the elf file
crcLocationElfPtrHex=$(printf '%x' $crcLocationElfPtr)

echo $crcLocationElfPtrHex: $crcHex > /tmp/crc
xxd -r /tmp/crc ${f_name}.elf



### Create the resulting bin and hex files

$TOOLPATH/arm-none-eabi-objcopy -O binary ${f_name}.elf  ${f_name}_crc.bin
fileLen=$(stat --printf="%s" ${f_name}_crc.bin)
echo _crc.bin filelen:  $fileLen >> log
#extend _crc.bin file to even multiple of 64 bytes
newFileLen=$(( ( ($fileLen / 64)+1) * 64 ))
add=$(( $newFileLen - $fileLen ))
addhex=$(printf '%d'  $add)
echo new file length and add: $newFileLen $add >> log
truncate --size=+$add ${f_name}_crc.bin
# convert bin file to intel hex file with 64 bytes per line 64*2+12 
srec_cat ${f_name}_crc.bin -binary -offset $isrVectorAbsAddr -output ${f_name}_crc.hex -Intel -line-length=140

### end










