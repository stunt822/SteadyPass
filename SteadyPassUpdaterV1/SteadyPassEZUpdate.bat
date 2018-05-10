setlocal
IF EXIST "%PROGRAMFILES(X86)%" (GOTO 64BIT) ELSE (GOTO 32BIT)

:64BIT
%~dp0files\drivers\dpinst64.exe /s
GOTO reset

:32BIT
%~dp0files\drivers\dpinst32.exe /s
GOTO reset

:reset
for /f "tokens=1* delims==" %%I in ('wmic path win32_pnpentity get caption /format:list ^| find "Arduino Leonardo"') do (
    call :resetCOM "%%~J"
)

:continue

for /f "tokens=1* delims==" %%I in ('wmic path win32_pnpentity get caption /format:list ^|find "Arduino Leonardo"')  do (
    call :setCOM "%%~J"
)

:: end main batch
goto :EOF

:resetCOM <WMIC_output_line>
:: sets _COM#=line
setlocal
set "str=%~1"
set "num=%str:*(COM=%"
set "num=%num:)=%"
set port=COM%num%
echo %port%
mode %port%: BAUD=1200 parity=N data=8 stop=1
goto :continue

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
%~dp0files\avrdude -v -C %~dp0files\avrdude.conf -patmega32u4 -cavr109 -P%port% -b57600 -D -V -Uflash:w:%~dp0files\SteadyPassV1PID.ino.hex:i