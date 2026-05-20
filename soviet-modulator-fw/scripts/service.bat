@echo off
echo Starting Modulator Service on COM8...

python .\scripts\modulator_service.py COM8 ^
    --rate 1000000 ^
    --crate 4 ^
    --rs 1 ^
    --conv 1 ^
    --fecf 0 ^
    --rand 1
pause
