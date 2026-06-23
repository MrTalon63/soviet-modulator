@echo off
echo Starting Modulator Service on COM8...

python modulator_service.py COM9 ^
    --rate 20000 ^
    --crate 4 ^
    --rs 1 ^
    --inter 4 ^
    --conv 1 ^
    --fecf 0 ^
    --rand 1 ^
    --randpoly 8 ^
    --rrc 1 ^
    --rrc-alpha 0.5 ^
    --rrc-span 0
pause