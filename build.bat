@echo off
rem Builds everything into build\ with MSVC (VS 2019+ Build Tools, x64).
setlocal
cd /d "%~dp0"
set "PATH=%PATH%;%ProgramFiles(x86)%\Microsoft Visual Studio\Installer"
where cl >nul 2>nul || call "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
if not exist build mkdir build
set CF=/nologo /std:c++17 /EHsc /O2 /W4 /MT /I third_party\scssdk /Fobuild\ /Fdbuild\

rem (tools\re\ holds the reverse-engineering probe and es_ctl; only sigscan is built.)
rem The road generator (C#, .NET 10): dotnet build tools\ets2_roads -c Release -o build\ets2_roads
cl %CF% ets2_plugin\eurospeed_plugin.cpp ets2_plugin\camhook.cpp ets2_plugin\truckpuppet.cpp ets2_plugin\fuel.cpp /LD /Febuild\eurospeed.dll /link /DEF:ets2_plugin\eurospeed.def || exit /b 1
cl %CF% bridge\lfs_bridge.cpp /Febuild\lfs_bridge.exe || exit /b 1
cl %CF% tools\fake_ets2.cpp /Febuild\fake_ets2.exe || exit /b 1
cl %CF% tools\fake_lfs.cpp /Febuild\fake_lfs.exe || exit /b 1
cl %CF% /await tools\overlay\overlay.cpp /Febuild\overlay.exe || exit /b 1
cl %CF% tools\lfs_insim.cpp /Febuild\lfs_insim.exe || exit /b 1
cl %CF% tools\wheeltest.cpp /Febuild\wheeltest.exe || exit /b 1
cl %CF% tests\codescan_test.cpp /Febuild\codescan_test.exe || exit /b 1
cl %CF% tools\re\sigscan.cpp /Febuild\sigscan.exe || exit /b 1
cl %CF% tests\mapping_test.cpp /Febuild\mapping_test.exe || exit /b 1
build\mapping_test.exe || exit /b 1
echo BUILD OK
