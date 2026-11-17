@echo off
rem Opens the TejoOne viewer in Edge (or Chrome) on this Windows computer.
set "PAGE=%~dp0index.html"
set "EDGE=%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe"
if not exist "%EDGE%" set "EDGE=%ProgramFiles%\Microsoft\Edge\Application\msedge.exe"
set "CHROME=%ProgramFiles%\Google\Chrome\Application\chrome.exe"
if not exist "%CHROME%" set "CHROME=%LocalAppData%\Google\Chrome\Application\chrome.exe"
if exist "%EDGE%" ( start "" "%EDGE%" "%PAGE%" & exit /b )
if exist "%CHROME%" ( start "" "%CHROME%" "%PAGE%" & exit /b )
start "" "%PAGE%"
