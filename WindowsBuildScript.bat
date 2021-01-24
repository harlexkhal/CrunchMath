rem Use this batch file to build CrunchMath for Visual Studio

rem Relpace this path with your cmake installation path:- location of cmake on your PC
set PATH="C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin";%PATH%

rmdir /s /q WindowsBuild
mkdir WindowsBuild
cd WindowsBuild
cmake ..

rem To force 32bit architecture builds
rem cmake ../ -A "Win32"

PAUSE