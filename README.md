# link-ecu-calibration

A C++17 ECU calibration core for Link ECU-style engine management:
2D fuel/VE and ignition-table interpolation, injector pulse width, duty cycle,
target lambda and lambda error, IAT/knock ignition corrections, rev-limit
protection, calibration validation and a generated Link-styled dashboard.

This is an independent reference implementation for Link ECU's calibration
workflow.

## Why this exists

An engine calibration has to stay coherent from table to injector. The tuner
interpolates VE and ignition advance at the current RPM and manifold pressure,
converts airflow into injector pulse width, checks duty cycle and lambda, applies
IAT/knock retard, and respects the rev limit. This core makes those calculations
explicit and testable.

## Formulas

```text
VE              = bilinear(fuelTable, rpm, mapKpa)
pulseMs         = deadTimeMs + (VE/100) * (MAP/100) * (14.7 / targetAFR) * injectorBaseMs
dutyPercent     = pulseMs / (120000 / rpm) * 100
targetLambda    = targetAFR / 14.7
lambdaError     = measuredLambda - targetLambda
iatCorrection   = max(0, (iatC - 20) / 10) * iatRetardDeg
finalIgnition   = clamp(bilinear(ignitionTable) - knockRetard - iatCorrection, 0, 50)
revLimitActive  = rpm >= revLimitRpm
```

## Build and test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The custom CTest runner covers table interpolation, clamping, pulse width, dead
time, duty cycle, target lambda, lambda error, IAT/knock corrections, rev limit,
validation errors and dashboard rendering.

## Dashboard

```bash
./build/link_ecu_calibration dashboard.html
```

The generated dashboard uses Link's purple `#3C1053`, red `#D22730`, Link
Display/Inclusive Sans fonts and 12px radius.

## Design tokens

- Heading: Link Display, sans-serif
- Body: Inclusive Sans, sans-serif
- Purple: `#3C1053`
- Deep purple: `#310E41`
- Red: `#D22730`
- Surface: `#F3EEF7`
- Background: `#FFFFFF`
- Radius: `12px`
- Spacing unit: `8px`

## Stack

C++17, CMake, CTest, GitHub Actions and a generated HTML/CSS dashboard.
