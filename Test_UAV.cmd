@echo off
setlocal
powershell.exe -NoProfile -File "%~dp0launch_project.ps1" -Mode Test
pause
endlocal
