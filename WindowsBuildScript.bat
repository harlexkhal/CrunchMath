rem Use this batch file to build CrunchMath for Visual Studio

rem Relpace this path with your cmake installation path:- location of cmake on your PC
set PATH="C:\Program Files (x86)\cmake-3.17.0-rc2-win32-x86\cmake-3.17.0-rc2-win32-x86\bin";%PATH%

rmdir /s /q WindowsBuild
mkdir WindowsBuild
cd WindowsBuild
cmake ..

rem To force 32bit architecture builds
rem cmake ../ -A "Win32"

PAUSE