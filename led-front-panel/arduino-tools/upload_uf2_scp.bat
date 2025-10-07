@echo off
setlocal

REM 1) Arg1 = full path to .uf2 built by Arduino
set "UF2=%~1"

if not exist "%UF2%" (
  echo [ERROR] UF2 not found: %UF2%
  exit /b 2
)

REM 2) Locate pscp.exe (prefer Program Files, else PATH)
set "PSCP=C:\arduino-tools\pscp.exe"
if not exist "%PSCP%" (
  for %%P in (pscp.exe) do (
    if not "%%~$PATH:P"=="" set "PSCP=%%~$PATH:P"
  )
)

if not exist "%PSCP%" (
  echo [ERROR] pscp.exe not found. Install PuTTY or add pscp.exe to PATH.
  exit /b 3
)

REM 3) Upload to the Radxa box
REM    -batch: no prompts; -q: quiet; -scp: force SCP protocol
"%PSCP%" -batch -q -scp -pw radxa "%UF2%" ^
  root@192.168.168.46:/root/uf2-dropbox/sketch.uf2

set ERR=%ERRORLEVEL%
if not "%ERR%"=="0" (
  echo [ERROR] Upload failed with code %ERR%
  exit /b %ERR%
)

echo Upload OK
exit /b 0