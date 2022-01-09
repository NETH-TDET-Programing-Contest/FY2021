rmdir /s /q venv

python -m venv ./venv
cd venv/Scripts
call activate
cd ../..
python -m pip install -r requirements.txt