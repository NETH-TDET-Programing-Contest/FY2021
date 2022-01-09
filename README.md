# FY2021 NETH CARLA Challenge

# CARLA Overnight Build 0.9.11 Download Link
https://carla-releases.s3.eu-west-3.amazonaws.com/Windows/CARLA_0.9.11.zip


# How to initial setup
1. Create python virtual environment by using Terminal (PowerShell or Command Prompt) at root of this folder  
`python -m venv ./venv`
2. Activate python virtual environment by using  
`./venv/Scripts/activate`  
`(venv)` will appear at the beginning of your Terminal like this  
`(venv) PS C:\Users\supav\Desktop\FY2021>`
3. Install library  
`python -m pip install -r requirements.txt`

# How to run agent
1. Run CARLA server by execute `start_carla_world.bat`
2. Open Terminal (PowerShell or Command Prompt) and activate python virtual environment by using  
`./venv/Scripts/activate`  
`(venv)` will appear at the beginning of your Terminal like this  
`(venv) PS C:\Users\supav\Desktop\FY2021>`
3. Run agent by execute `./run_agent.bat`

# How to config run_agent scripts
run_agent.bat scripts
```
@echo off
title Exeute agent

set DIR_ROOT=%CD%
set CARLA_ROOT=%DIR_ROOT%\carla
set SCENARIO_RUNNER_ROOT=%DIR_ROOT%\scenario_runner
set LEADERBOARD_ROOT=%DIR_ROOT%\leaderboard
set TRACKS_ROOT=%DIR_ROOT%\tracks

set PYTHONPATH=%CARLA_ROOT%;%PYTHONPATH%
set PYTHONPATH=%CARLA_ROOT%\dist\carla-0.9.11-py3.7-win-amd64.egg;%PYTHONPATH%
set PYTHONPATH=%SCENARIO_RUNNER_ROOT%;%PYTHONPATH%
set PYTHONPATH=%LEADERBOARD_ROOT%;%PYTHONPATH%

set PYTHON=%DIR_ROOT%\venv\Scripts\python.exe

PYTHON "%LEADERBOARD_ROOT%\leaderboard\leaderboard_evaluator.py" ^
--routes="%TRACKS_ROOT%\track_01.xml" ^
--scenarios="%TRACKS_ROOT%\track_01.json" ^
--agent="%DIR_ROOT%\src\dummy_agent.py" ^
--agent-config="" ^
--debug=1

pause
```

1. `--routes` is path to generate routes file. (Staff will provide this file)
2. `--scenarios` is path to to generate scenarios or events file. (Staff will provide this file)
3. `--agent` is path to your agent main source code.
4. `--agent-config` is path to your agent config file.
5. `--debug` (1:Enable, 0:Disable) waypoints drawing in CARLA world

# Run result definition
```
←[1m========= Results of RouteScenario_0 (repetition 0) ------ ←[91mFAILURE←[0m ←[1m=========←[0m

╒═════════════════════════════════╤═════════════════════╕
│ Start Time                      │ 2021-05-21 12:22:36 │
├─────────────────────────────────┼─────────────────────┤
│ End Time                        │ 2021-05-21 12:24:26 │
├─────────────────────────────────┼─────────────────────┤
│ Duration (System Time)          │ 110.05s             │
├─────────────────────────────────┼─────────────────────┤
│ Duration (Game Time)            │ 154.4s              │
├─────────────────────────────────┼─────────────────────┤
│ Ratio (System Time / Game Time) │ 1.403               │
╘═════════════════════════════════╧═════════════════════╛

╒═══════════════════════╤═════════╤═════════╕
│ Criterion             │ Result  │ Value   │
├───────────────────────┼─────────┼─────────┤
│ RouteCompletionTest   │ ←[91mFAILURE←[0m │ 32.26 % │
├───────────────────────┼─────────┼─────────┤
│ OutsideRouteLanesTest │ ←[91mFAILURE←[0m │ 1.3 %   │
├───────────────────────┼─────────┼─────────┤
│ CollisionTest         │ ←[91mFAILURE←[0m │ 6 times │
├───────────────────────┼─────────┼─────────┤
│ RunningRedLightTest   │ ←[91mFAILURE←[0m │ 3 times │
├───────────────────────┼─────────┼─────────┤
│ RunningStopTest       │ ←[91mFAILURE←[0m │ 2 times │
├───────────────────────┼─────────┼─────────┤
│ InRouteTest           │ ←[91mFAILURE←[0m │         │
├───────────────────────┼─────────┼─────────┤
│ AgentBlockedTest      │ ←[92mSUCCESS←[0m │         │
├───────────────────────┼─────────┼─────────┤
│ Timeout               │ ←[92mSUCCESS←[0m │         │
╘═══════════════════════╧═════════╧═════════╛
```
TBD.
