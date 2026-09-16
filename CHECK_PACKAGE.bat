@echo off
cd /d "%~dp0"
set BAD=0
for %%F in (".github\workflows\build-windows.yml" "src\autoseam_worker.cpp" "scripts\patch_optcuts_cmake.py" "Rotate_UV_PRO_NATIVE_AUTO_SEAM_V1.ms" "Rotate_UV_PRO_NATIVE_AUTO_SEAM_V1.mcr") do (
 if exist %%F (echo OK: %%F) else (echo MISSING: %%F& set BAD=1)
)
if "%BAD%"=="0" echo Package structure is complete.
pause
exit /b %BAD%
