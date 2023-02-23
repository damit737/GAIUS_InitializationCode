@echo on

cd ..\Doc
copy ..\Debug\Application.hex ..\Doc\Application.hex

srec_cat.exe Application.hex -Intel ^
 -crop 0x08020000 0x08100000 ^
 -offset -0x08020000 ^
 -o Application.bin -binary
 
srec_cat.exe Application.hex -Intel ^
 -crop 0x90221000 0x91000000 ^
 -offset -0x90221000 ^
 -o Resource.bin -binary
 
::工控
srec_cat Application.bin -binary -offset 0x08020000 ^
 Bootloader.bin -binary -offset 0x08000000 ^
 Resource.bin -binary -offset 0x90221000 ^
 -o GAIUS_WL0F00050000FGAAASB00.hex -Intel 

del Application.hex
