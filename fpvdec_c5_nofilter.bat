@echo off
rem ==========================================================
rem  fpvdec emulating the C5VRX-5 receive chain without the channel filter (as C5VRX-5 / U85Q):
rem  digital Direct Gain V5 + 4-bit ultrafine IQ before the demodulator
rem  (--c5-nofilter). One "C5:" status line per second in this window.
rem  Extra arguments are passed through, e.g.
rem    fpvdec_c5_nofilter.bat --channel E1
rem    fpvdec_c5_nofilter.bat --channel F3 --c5-maxgain 10
rem  Keys: a = next channel, h = help, q = quit
rem ==========================================================
setlocal
set "EXE_DIR=%~dp0build\Release"
if not exist "%EXE_DIR%\fpvdec.exe" goto :no_exe

rem Recordings and screenshots land next to the exe, as before.
cd /d "%EXE_DIR%"
"%EXE_DIR%\fpvdec.exe" --c5-nofilter %*
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
