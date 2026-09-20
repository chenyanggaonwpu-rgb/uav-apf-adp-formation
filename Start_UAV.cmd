@echo off
setlocal
powershell.exe -NoProfile -File "%~dp0launch_project.ps1" -Mode Run
if errorlevel 1 pause
endlocal
