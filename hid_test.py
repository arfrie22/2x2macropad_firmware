# Install python3 HID package https://pypi.org/project/hid/
import hid

# default is TinyUSB (0xcafe), Adafruit (0x239a), RaspberryPi (0x2e8a), Espressif (0x303a) VID
USB_VID = (0x1209, 0xcafe, 0x239a, 0x2e8a, 0x303a)

print("VID list: " + ", ".join('%02x' % v for v in USB_VID))

for vid in  USB_VID:
    for dict in hid.enumerate(vid):
        if dict['interface_number'] == 2:
            print(dict)
            dev = hid.Device(dict['vendor_id'], dict['product_id'])
            while dev:
                # while True:
                    # Get input from console and encode to UTF8 for array of chars.
                    # hid generic inout is single report therefore by HIDAPI requirement
                    # it must be preceeded with 0x00 as dummy reportID
                    # str_out = b'\x00'
                    # str_out += input("Send text to HID Device : ").encode('utf-8')
                    # dev.write(str_out)
                # dev.write(bytes(list(range(64))))
                # dev.write(b'\xFE' * 64)
                # dev.write(b'\xFF' * 64)
                # str_out = b'\x04' + b'\x00' + b'\x00' * 62
                str_out = b'\x04\x00\xc98ts'
                print("Sending to HID Device:", str_out, '\n')
                dev.write(str_out)
                str_in = dev.read(64, 500)
                print("Received from HID Device:", str_in, '\n')
            dev.close()
            