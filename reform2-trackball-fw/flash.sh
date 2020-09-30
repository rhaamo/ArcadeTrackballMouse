

dfu-programmer atmega32u2 erase --suppress-bootloader-mem 

dfu-programmer atmega32u2 flash ./Mouse.hex --suppress-bootloader-mem 

dfu-programmer atmega32u2 start 
