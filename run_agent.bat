@echo off
title Exeute agent

set DIR_ROOT=%CD%
set CARLA_ROOT=%DIR_ROOT%\carla
set SCENARIO_RUNNER_ROOT=%DIR_ROOT%\scenario_runner
set LEADERBOARD_ROOT=%DIR_ROOT%\leaderboard

set PYTHONPATH=%CARLA_ROOT%;%PYTHONPATH%
set PYTHONPATH=%CARLA_ROOT%\dist\carla-0.9.11-py3.7-win-amd64.egg;%PYTHONPATH%
set PYTHONPATH=%SCENARIO_RUNNER_ROOT%;%PYTHONPATH%
set PYTHONPATH=%LEADERBOARD_ROOT%;%PYTHONPATH%

set SCENARIOS="%LEADERBOARD_ROOT%\data\all_towns_traffic_scenarios_public.json"
set ROUTES="%LEADERBOARD_ROOT%\data\routes_devtest.xml"
set REPETITIONS=1
set DEBUG_CHALLENGE=1
set TEAM_AGENT="%DIR_ROOT%\src\human_agent.py"
set CHECKPOINT_ENDPOINT="%LEADERBOARD_ROOT%\results.json"
set CHALLENGE_TRACK_CODENAME=SENSORS

.\venv\Scripts\python.exe ".\leaderboard\leaderboard\leaderboard_evaluator.py" ^
--scenarios=%SCENARIOS% ^
--routes=%ROUTES% ^
--repetitions=%REPETITIONS% ^
--track=%CHALLENGE_TRACK_CODENAME% ^
--checkpoint=%CHECKPOINT_ENDPOINT% ^
--agent=%TEAM_AGENT% ^
--agent-config=%TEAM_CONFIG% ^
--debug=%DEBUG_CHALLENGE% ^
--record=%RECORD_PATH% ^
--resume=%RESUME%

pause