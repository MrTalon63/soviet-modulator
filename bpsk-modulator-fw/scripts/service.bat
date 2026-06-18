@echo off
echo Starting Modulator Service on COM8...

python .\scripts\modulator_service.py COM7 ^
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
    --rrc-min-dac 1500000 ^
    --rrc-span 6 ^
    --rrc-type 1
pause
