# Debug Log — PMSM FOC Simulink Build (Step 5a–5d)

> Collected errors, root causes, and fixes encountered during programmatic Simulink model generation.

---

## 1. Port Indexing Errors

### Problem: Ports numbered 49–55 instead of 1–7
When deleting the default `In1`/`Out1` from a Subsystem and adding new Inport/Outport blocks, Simulink continues auto-incrementing from the deleted block's port number.

**Root Cause**: `delete_block` removes the block but does not reset the port counter.

**Fix**: Always explicitly set the `Port` parameter after adding new ports:
```matlab
add_block('simulink/Sources/In1', [path '/MyInport']);
set_param([path '/MyInport'], 'Port', '1');  % MUST reset explicitly
```

**Verification**:
```matlab
inports = find_system(sys, 'SearchDepth', 1, 'BlockType', 'Inport');
for i = 1:length(inports)
    fprintf('%s -> Port %s\n', get_param(inports{i}, 'Name'), get_param(inports{i}, 'Port'));
end
```

---

## 2. Block Positioning Errors (Overlapping Blocks)

### Problem: All blocks stacked at [0 0 30 30]
Simulink's default block position when added via `add_block()` is tiny and overlapping.

**Root Cause**: Not specifying the `Position` parameter.

**Fix**: Use a dynamic grid-based positioning function:
```matlab
W=100; H=60; HG=150; VG=100; x0=30; y0=30;
bx = @(c,r) [x0+c*(HG+W), y0+r*(VG+H), x0+c*(HG+W)+W, y0+r*(VG+H)+H];
```
Then every `add_block` call includes `'Position', bx(col, row)`.

---

## 3. Block Library Name Errors

### Problem: `There is no block named 'simulink/Math Operations/Saturation'`

**Fix**: The correct library path is `'simulink/Discontinuities/Saturation'`.

| Intuitive Name | Actual Library Path |
|----------------|---------------------|
| `Saturation` | `simulink/Discontinuities/Saturation` |
| `Fcn` | `simulink/User-Defined Functions/Fcn` |

---

## 4. Cell Array Variable Assignment

### Problem: `Error using assignin — Argument must be a text scalar`

```matlab
% BROKEN: iterating over a cell array directly
for v = {'a',1; 'b',2}
    assignin('base', v{1}, v{2});  % v is a 2x2 cell, not a single row
end
```

**Root Cause**: MATLAB's `for` loop over a matrix iterates by columns, not rows.

**Fix**:
```matlab
vars = {'a',1; 'b',2};
for vi = 1:size(vars, 1)
    assignin('base', vars{vi, 1}, vars{vi, 2});
end
```

---

## 5. Fcn Block Expression Syntax

### Problem: `The expression: mod(u(1)+pi, 2*pi)-pi has a syntax error`

The Simulink Fcn block uses `u` (not `u(1)`) as the input variable. However, the expression `mod(u+pi, 2*pi)-pi` also failed parsing.

**Root Cause**: The `mod` function with two arguments seems to confuse the Fcn block parser in certain MATLAB versions.

**Fix**: Use three separate Fcn blocks chained together:
```matlab
% Block 1: AddPi  — expr = 'u+pi'
% Block 2: Mod2Pi — expr = 'u - 2*pi*floor(u/(2*pi))'  (manual modulo)
% Block 3: SubPi  — expr = 'u-pi'
```

This avoids the `mod()` function entirely and uses `floor()` for the modulo operation.

---

## 6. Solver Selection

### Problem: Unstable simulation or slow convergence with variable-step solvers

**Fix**: Use fixed-step solver for motor control simulations:
```matlab
set_param(modelName, 'Solver', 'ode4', 'SolverType', 'Fixed-step', ...
    'FixedStep', '1e-5', 'StopTime', '0.2');
```
- `ode4` (RK4) is a good balance of accuracy and speed
- `1e-5` step size handles the 500 Hz current loop adequately

---

## 7. Wiring Block Names vs. Port Numbers

### Problem: `add_line` fails with "unable to locate port"

**Root Cause**: Confusing block name with port number. When a Subsystem has output ports, they are referenced by their `Port` number, not the block's name.

**Fix**: Subsystem output ports are accessed as `'SubsystemName/PortNumber'`:
```matlab
% WRONG (tries to find block named 'CC'):
add_line(testName, 'CC/1', 'Motor Plant/1');

% RIGHT (subsystem name 'Current Controller', port 1):
add_line(testName, 'Current Controller/1', 'Motor Plant/1');
```

---

## 8. Model Modification State

### Problem: `Warning: Unable to close the model ... because it has been changed.`

**Fix**: Always close with discard option before regenerating:
```matlab
if bdIsLoaded(modelName)
    close_system(modelName, 0);  % 0 = discard changes
end
```

---

## 9. File Existence Check

### Problem: `delete` fails on non-existent file

**Fix**: Always check existence or use try-catch:
```matlab
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end
```

---

## 10. Test Harness Verification

### Guideline: After building each test harness, **simulate and verify** before reporting success.

Expected steady-state accuracies for this project:
| Loop | Signal | Setpoint | Measured | Error Threshold |
|------|--------|----------|----------|-----------------|
| Current | Id | 1.0 A | 1.0000 A | < 0.1 A |
| Current | Iq | 2.0 A | 2.0000 A | < 0.1 A |
| Speed | ω | 100 rad/s | 100.07 rad/s | < 5 rad/s |
| Position | θ | π rad | 3.000 rad (wrapped) | < 0.5 rad |

---

## Summary of Debugging Workflow

When a build script fails:

1. **Check the error line number** — read the script at that line
2. **Is it a library path?** → Verify in MATLAB: `find_system('simulink', 'SearchDepth',2, 'Name','Saturation')`
3. **Is it port indexing?** → Add explicit `set_param(..., 'Port', ...)` after every `add_block`
4. **Is it position overlap?** → Use the `bx()` function consistently
5. **Is it wiring?** → Verify subsystem port numbers match `add_line` references
6. **Is it a cell array?** → Use index-based loop, not direct iteration
7. **Is it a syntax error in Fcn block?** → Avoid `mod()`, use `floor()` approach
