@echo off

REM Remove *.asv files generated by MATLAB in this directory and all subdirectories.

echo Removing "*.asv" files...
for /f "delims==" %%i in ('dir /b /on /s "%~p0*.asv"') do del "%%i" /f /q
echo.

echo "%~n0.bat" done.


