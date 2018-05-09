%~dp0\avrdude -C %~dp0\avrdude.conf -v -patmega32u4 -cavr109 -PCOM1 -b57600 -D -Uflash:w:"%~dp0/SteadyPassV1PID.ino.hex:i
