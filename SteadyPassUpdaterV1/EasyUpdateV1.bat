%~dp0\files\avrdude -C %~dp0\files\avrdude.conf -v -patmega32u4 -cavr109 -PCOM1 -b57600 -D -Uflash:w:"%~dp0\files\SteadyPassV1PID.ino.hex:i
