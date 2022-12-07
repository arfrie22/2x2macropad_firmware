$PICO_SDK_PATH/build/elf2uf2/elf2uf2 $1 $1.uf2

# scan every usb device 
# if the device is a pico, copy the uf2 file to it
# if the device is not a pico, skip it
# for mac

found=0

if [ "$(uname)" == "Darwin" ]; then
    for device in /Volumes/*; do
        if [ -e "$device/INFO_UF2.TXT" ]; then
            echo "Found Pico at $device"
            cp $1.uf2 $device/out.uf2
            found=1
        fi
    done
fi

# if [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
#     for device in /media/*; do
#         if [ -e "$device/INFO_UF2.TXT" ]; then
#             echo "Found Pico at $device"
#             cp $1.uf2 $device/out.uf2
#             found=1
#         fi
#     done
# fi

if [ $found -eq 0 ]; then
    echo "No Pico found"
fi