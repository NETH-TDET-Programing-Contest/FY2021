@echo off
title Exeute agent

set DIR_ROOT=%CD%\..\..
set CARLA_ROOT=%DIR_ROOT%\carla
set SCENARIO_RUNNER_ROOT=%DIR_ROOT%\scenario_runner
set LEADERBOARD_ROOT=%DIR_ROOT%\leaderboard
set TRACKS_ROOT=%DIR_ROOT%\tracks

set PYTHONPATH=%CARLA_ROOT%;%PYTHONPATH%
set PYTHONPATH=%CARLA_ROOT%\dist\carla-0.9.11-py3.7-win-amd64.egg;%PYTHONPATH%
set PYTHONPATH=%SCENARIO_RUNNER_ROOT%;%PYTHONPATH%
set PYTHONPATH=%LEADERBOARD_ROOT%;%PYTHONPATH%

set PYTHON=%DIR_ROOT%\venv\Scripts\python.exe

cd %DIR_ROOT%\venv\Scripts
call activate

PYTHON "%LEADERBOARD_ROOT%\leaderboard\leaderboard_evaluator.py" ^
--routes="%TRACKS_ROOT%\track_01.xml" ^
--scenarios="%TRACKS_ROOT%\track_01.json" ^
--agent="%DIR_ROOT%\src\tutorial\agent_24.py" ^
--agent-config="" ^
--debug=1

pause