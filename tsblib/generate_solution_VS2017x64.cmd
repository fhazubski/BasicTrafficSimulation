REM rmdir /Q /S build
REM if errorlevel 1 echo Unsuccessful
mkdir build
cd build
cmake .. -G "Visual Studio 15 2017 Win64"
pause
