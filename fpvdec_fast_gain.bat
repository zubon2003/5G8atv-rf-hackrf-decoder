@echo off
rem ==========================================================
rem  fpvdec with the C5VRX-style fast gain control (--gain fast)
rem  Extra arguments are passed through, e.g.
rem    fpvdec_fast_gain.bat --channel E1
rem    fpvdec_fast_gain.bat --freq 5806e6 --invert
rem  Keys: a = gain AUTO -> FAST -> MANUAL, h = help, q = quit
rem ==========================================================
setlocal
set "EXE_DIR=%~dp0build\Release"
if not exist "%EXE_DIR%\fpvdec.exe" goto :no_exe

rem Recordings and screenshots land next to the exe, as before.
cd /d "%EXE_DIR%"
"%EXE_DIR%\fpvdec.exe" --gain fast %*
if errorlevel 1 goto :failed
endlocal
exit /b 0

:no_exe
echo.
echo [ERROR] build\Release\fpvdec.exe not found. Build it first:
echo   cmake -B build -G "Visual Studio 17 2022" -A x64
echo   cmake --build build --config Release
echo.
pause
endlocal
exit /b 1

:failed
echo.
echo [ERROR] fpvdec exited with an error (see the messages above).
echo   Check that the HackRF One is connected (hackrf_info).
echo.
pause
endlocal
exit /b 1
