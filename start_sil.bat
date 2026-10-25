@echo off
rem Software-in-the-loop demo: starts the controller process, the telemetry
rem bridge and the 3D view. Then, in your own MATLAB (folder matlab\), run e.g.
rem   ctrl_ext = 1; sil_lockstep = 0; viz_on = 1;
rem   load_system('hydrofoil_sim'); set_param('hydrofoil_sim', 'EnablePacing', 'on'); sim('hydrofoil_sim')
rem or run_sil_demo.m.
cd /d "%~dp0"

rem MATLAB executable: on the PATH, or set ML to its full path here
set "ML=matlab"
where matlab >nul 2>nul || set "ML=D:\matlab\MAIN\bin\matlab.exe"
if not "%ML%"=="matlab" if not exist "%ML%" (
    echo MATLAB not found. Edit the line "set ML=..." in start_sil.bat.
    pause
    exit /b 1
)
set "PY=python"
where python >nul 2>nul || set "PY=py"

start "ctrl_server" "%ML%" -batch "cd('matlab'); ctrl_server"
start "sil_bridge" %PY% viz\sil_bridge.py
timeout /t 2 >nul
start http://localhost:8000
echo Wait until the ctrl_server window shows "Controller listening on port 5501" (10-30 s),
echo then run run_sil_demo (or run_sil) in your MATLAB.
