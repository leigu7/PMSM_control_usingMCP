# Simulink Build Rules — PMSM FOC Project

> Consolidated build rules, architecture, and quick-start reference.

---

## 1. CRITICAL: Port Indexing

1. **Simulink port indexing is strictly 1-based.** The first Inport is always `Port 1`, the first Outport is always `Port 1`. There is **no Port 0**.

2. When using `add_line(sys, 'SrcBlock/SrcPort', 'DstBlock/DstPort')`, the port numbers must exactly match the block's 1-based `Port` parameter.

3. When building a Subsystem:
   - `add_block('simulink/Ports & Subsystems/Subsystem', ...)` auto-creates `In1` (Port=1) and `Out1` (Port=1).
   - If you delete these defaults and add your own Inport/Outport blocks, **always explicitly set the `Port` parameter** to reset numbering:
     ```matlab
     set_param([path '/MyInport'], 'Port', '1');
     ```

4. If port numbers are not explicitly reset, they will continue auto-incrementing from the deleted block's port number, resulting in ports numbered 49-55 instead of 1-7.

5. To verify correct port numbering:
   ```matlab
   inports = find_system(sys, 'SearchDepth', 1, 'BlockType', 'Inport');
   for i = 1:length(inports)
       fprintf('%s -> Port %s\n', get_param(inports{i}, 'Name'), get_param(inports{i}, 'Port'));
   end
   ```

---

## 2. CRITICAL: Block Positioning

1. **NEVER leave blocks at their default positions.** Stacking blocks on top of each other is strictly forbidden.

2. **Maintain a strict "Left-to-Right" data flow grid.** Signal flow must go from left to right.

3. **Dynamically calculate the `Position` vector** `[left, top, right, bottom]` for every block using offset variables (e.g., `col`, `row`, `bx`, `by`).

4. **Standard block size**: `[100, 60]` (width=100, height=60).

5. **Spacing rules**:
   - Horizontal spacing between sequential blocks: **≥ 150 pixels**
   - Vertical spacing between parallel branches: **≥ 100 pixels**

6. **Recommended layout algorithm**:
   ```matlab
   width = 100; height = 60;
   hgap = 150; vgap = 100;
   x0 = 30; y0 = 30;
   bx = @(col, row) [x0 + col*(hgap+width), y0 + row*(vgap+height), ...
                     x0 + col*(hgap+width) + width, y0 + row*(vgap+height) + height];
   ```

7. **Subsystem sizing**: Subsystem blocks should be large enough to contain all internal blocks with padding.

---

## 3. Wiring Verification

Always verify connections after building a model:

```matlab
lines = find_system(sys, 'FindAll', 'on', 'Type', 'line');
for i = 1:length(lines)
    h = lines(i);
    src = get_param(h, 'SrcBlockHandle');
    dst = get_param(h, 'DstBlockHandle');
    src_port = get_param(get_param(h, 'SrcPortHandle'), 'PortNumber');
    dst_port = get_param(get_param(h, 'DstPortHandle'), 'PortNumber');
    fprintf('%s[%d] -> %s[%d]\n', get_param(src,'Name'), src_port, get_param(dst,'Name'), dst_port);
end
```

---

## 4. PMSM Plant — Cross-Coupling Wiring

For the d-axis cross-coupling `+we*Lq*Iq`:
- `weCalc[1] -> weLq[1]` (we to product port 1)
- `Lq_c[1] -> weLq[2]` (Lq to product port 2)
- `weLq[1] -> weLqIq[1]` (we*Lq to product port 1)
- `IntIq[1] -> weLqIq[2]` (Iq to product port 2)
- `weLqIq[1] -> SumD[3]` (result to Sum D port 3, which is +)

For the q-axis back-EMF `-(we*Ld*Id + we*psid)`:
- `weCalc[1] -> weLd[1]` (we to product port 1)
- `Ld_c[1] -> weLd[2]` (Ld to product port 2)
- `weLd[1] -> weLdId[1]` (we*Ld to product port 1)
- `IntId[1] -> weLdId[2]` (Id to product port 2)
- `weCalc[1] -> wePsid[1]` (we to product port 1)
- `psid_c[1] -> wePsid[2]` (psid to product port 2)
- `weLdId[1] -> SumEMF[1]` (sum port 1)
- `wePsid[1] -> SumEMF[2]` (sum port 2)
- `SumEMF[1] -> SumQ[3]` (to Sum Q port 3, which is -)

---

## 5. Project Architecture

### Control Cascade

```
θ_ref → [Position PI] → ω_ref → [Speed PI] → Iq_ref → [Current PI] → Vdq → [SVPWM] → Plant → θ, ω, Id, Iq
         5 Hz                     30 Hz                    500 Hz
```

### Bandwidth Hierarchy

| Loop | Bandwidth | Purpose |
|------|-----------|---------|
| Current (inner) | 500 Hz | Fast current tracking, decoupling |
| Speed (middle)  | 30 Hz  | Acceleration/deceleration, load rejection |
| Position (outer)| 5 Hz   | Smooth reference tracking, accuracy |

### File Inventory

| File | Purpose |
|------|---------|
| `pmsm_plant.slx` | Motor plant subsystem (standalone) |
| `pmsm_current_control.slx` | Current PI + decoupling |
| `pmsm_speed_control.slx` | Speed PI + Iq saturation |
| `pmsm_position_control.slx` | Position PI + angle wrapping |
| `test_current.slx` | Current loop test harness |
| `test_speed.slx` | Speed loop test harness |
| `test_position.slx` | Position loop test harness |
| `step5a_plant.m` | Build script for plant |
| `step5b_current_control.m` | Build script for current control |
| `step5c_speed_control.m` | Build script for speed control |
| `step5d_position_control.m` | Build script for position control |

---

## 6. Quick Start

### Run all verification tests

```matlab
% Step 1: Plant
run('step5a_plant.m')

% Step 2: Current control
run('step5b_current_control.m')

% Step 3: Speed control
run('step5c_speed_control.m')

% Step 4: Position control (full FOC)
run('step5d_position_control.m')
```

### Verification results (expected)

| Test | Input | Measured | Error |
|------|-------|----------|-------|
| Current | Id_ref=1A, Iq_ref=2A | Id=1.000A, Iq=2.000A | <0.1A |
| Speed | ω_ref=100 rad/s | ω=100.07 rad/s | <5 rad/s |
| Position | θ_ref=π rad | θ=3.005 rad | <0.5 rad |

---

## 7. Subsystem Port Maps

### Motor Plant (`pmsm_plant.slx`)
```
In[1]  Vd            Out[1] Id
In[2]  Vq            Out[2] Iq
In[3]  LoadTorque    Out[3] Te
In[4]  PolePairs     Out[4] omega_m
In[5]  J             Out[5] theta_m
In[6]  B
In[7]  Temperature
```

### Current Controller (`pmsm_current_control.slx`)
```
In[1] Id_ref    Out[1] Vd_ref
In[2] Iq_ref    Out[2] Vq_ref
In[3] Id_fb
In[4] Iq_fb
In[5] we
In[6] Ld
In[7] Lq
In[8] psid
```

### Speed Controller (`pmsm_speed_control.slx`)
```
In[1] omega_ref    Out[1] Iq_ref
In[2] omega_fb
```

### Position Controller (`pmsm_position_control.slx`)
```
In[1] theta_ref    Out[1] omega_ref
In[2] theta_fb
```

---

## 8. SVPWM Algorithm

1. **Voltage limiting**: `Vmag = sqrt(Vd² + Vq²)`. If `Vmag > Vmax`, scale: `Vd *= Vmax/Vmag`, `Vq *= Vmax/Vmag`.
2. **Inverse Park**: `Vα = Vd·cos(θe) - Vq·sin(θe)`, `Vβ = Vd·sin(θe) + Vq·cos(θe)`.
3. **Inverse Clarke**: `Va = Vα`, `Vb = -0.5·Vα + √3/2·Vβ`, `Vc = -0.5·Vα - √3/2·Vβ`.
