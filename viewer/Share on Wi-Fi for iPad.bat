@echo off
rem Shares the TejoOne viewer on the Wi-Fi so an iPhone / iPad can open it (see serve.py).
cd /d "%~dp0"
where py >nul 2>nul && ( py serve.py %* & goto :end )
where python >nul 2>nul && ( python serve.py %* & goto :end )
echo Python is not installed. Get it from https://www.python.org/downloads/ and run this again.
:end
pause
