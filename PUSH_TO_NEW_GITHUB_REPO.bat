@echo off
setlocal
cd /d "%~dp0"
echo.
echo RotateUV Native Auto Seam - push this COMPLETE package to a NEW empty GitHub repository
echo.
where git >nul 2>nul
if errorlevel 1 (
  echo ERROR: Git for Windows is not installed or not on PATH.
  pause
  exit /b 1
)
set /p REPO_URL=Paste the NEW empty GitHub repository HTTPS URL: 
if "%REPO_URL%"=="" exit /b 1
if not exist .git git init || goto :fail
git add . || goto :fail
git diff --cached --quiet
if errorlevel 1 git commit -m "Initial RotateUV Native Auto Seam complete package" || goto :commitfail
git branch -M main || goto :fail
git remote remove origin >nul 2>nul
git remote add origin "%REPO_URL%" || goto :fail
git push -u origin main || goto :fail
echo.
echo SUCCESS. Now open GitHub Actions and run Build Windows Auto Seam Worker.
pause
exit /b 0
:commitfail
echo Configure Git identity if requested, then run this BAT again.
echo git config --global user.name "Your Name"
echo git config --global user.email "you@example.com"
pause
exit /b 1
:fail
echo A Git command failed. Read the message above and retry.
pause
exit /b 1
