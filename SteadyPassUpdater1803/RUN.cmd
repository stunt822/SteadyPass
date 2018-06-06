setlocal 
IF EXIST "%PROGRAMFILES(X86)%" (GOTO 64BIT) ELSE (GOTO 32BIT)

:64BIT
%~dp0files\drivers\dpinst64.exe /s
GOTO reset

:32BIT
%~dp0files\drivers\dpinst32.exe /s
GOTO reset

:reset
for /f "tokens=1* delims==" %%I in ('wmic path win32_pnpentity get caption /format:list ^| find "Arduino"') do (
    call :resetCOM "%%~J"
)

:continue
for /f "tokens=1* delims==" %%I in ('wmic path win32_pnpentity get caption /format:list ^|find "Arduino"')  do (
    call :setCOM "%%~J"
	
)

:: end main batch
exit

:resetCOM <WMIC_output_line>
:: sets _COM#=line
setlocal
set "str=%~1"
set "num=%str:*(COM=%"
set "num=%num:)=%"
set port=COM%num%
echo %port%
mode %port%: BAUD=1200 parity=N data=8 stop=1
ping 127.0.0.1 -n 3 > nul
GOTO continue

:setCOM <WMIC_output_line>
:: sets _COM#=line
setlocal
set "str=%~1"
set "num=%str:*(COM=%"
set "num=%num:)=%"
set port=COM%num%
echo %port%
goto :flash


:flash
%~dp0files\avrdude -v -C %~dp0files\avrdude.conf -patmega32u4 -cavr109 -P%port% -b57600 -D -V -Uflash:w:%~dp0files\SteadyPassV18.ino.hex:i
echo off
echo "                                      __  o                           "
echo "                                     /  |/                            "
echo "                                   _/___|___________                  "
echo "                                 /  _______      __\                  "
echo " _______                        /  /_o_||__|    |                     "
echo "  \_\_\_\______________________/___             |                     "
echo "           \                       \____________|_____________        "
echo "            \        ________  __ _  ___  / ___ / /____        |      "
echo "             \      / ___/ _ \/  ' \/ _ \/ / -_/ __/ -_/    ___|      "
echo "              \     \___/\___/_/_/_/ .__/_/\__/\__/\__/     |         "
echo "               \                  /_/                      /          "
echo "^^^^^^^^^^^^^^^ \________________________________________ /_^^^^^^^^  " 
echo " ^^^^  ^^^^                                              \__|==| ^^   "
echo "^^         ^^^^^^^^       ^^^^ ^^^ ^^^^^      ^^^^^^^^^^ ^      ^^^^  "
echo "^^   ^^^^          ^^^^^^^^^^^^          ^^^^     ^^       ^^^^^      "
pause