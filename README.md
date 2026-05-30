# PMSM Field-Oriented Control — Incremental Build

This project builds a complete **Permanent Magnet Synchronous Motor (PMSM)** 
**Field-Oriented Control (FOC)** system step by step using MATLAB scripts.

## Approach

Each step validates one layer of the cascade control architecture before moving to the next:

| Step | Script | What It Builds |
|------|--------|----------------|
| 1 | `step1_build_plant_openloop.m` | Motor plant model with LUTs |
| 2 | `step2_build_current_loop_svpwm.m` | Current PI controllers + SVPWM |
| 3 | `step3_build_speed_loop.m` | Speed PI controller |
| 4 | `step4_build_position_loop.m` | Position PI controller (full cascade) |
| 5 | *(next)* | Simulink `.slx` model generation |

## Quick Start

```matlab
% Run all 4 steps sequentially
step1_build_plant_openloop
step2_build_current_loop_svpwm
step3_build_speed_loop
step4_build_position_loop
```

## Architecture

```
θ_ref → [PI] → ω_ref → [PI] → Iq_ref → [PI] → Vdq → [SVPWM] → Plant → θ, ω, Id, Iq
        5 Hz           30 Hz           500 Hz
```

## Files

- `SurfaceMountedPMSM.m` — Motor model class (LUT-based Ld, Lq, psid, psiq, Rs)
- `step1_build_plant_openloop.m` → `step4_build_position_loop.m` — Build scripts
- `BUILD_PROGRESS.md` — Detailed build summary with results

## Documentation

See `BUILD_PROGRESS.md` for complete build status, gain tables, and test results.
