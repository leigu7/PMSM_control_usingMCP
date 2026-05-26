function generate_surface_mounted_pmsm_foc_model()
% generate_surface_mounted_pmsm_foc_model  Create a Simulink FOC model for a surface-mounted PMSM.
%
% This script generates a model with 2D Ld/Lq/psi lookup tables and a
% temperature-dependent Rs lookup, plus an outer position loop, speed loop,
% and inner dq current loops.

    modelName = 'surface_mounted_pmsm_foc';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end

    new_system(modelName, 'Model');
    open_system(modelName);

    set_param(modelName, 'Solver', 'ode45', 'StopTime', '1', 'MaxStep', '1e-4', ...
        'SaveOutput', 'off', 'SaveState', 'off', 'SignalLogging', 'off');

    % Top-level sources and signals
    add_block('simulink/Sources/Sine Wave', [modelName '/PositionRef'], 'Position', [50 50 120 90], ...
        'Amplitude', '2*pi', 'Frequency', '1', 'SampleTime', '0');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/IdRef'], 'Position', [50 150 120 190], 'Value', '0');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/Temperature'], 'Position', [50 220 120 260], 'Value', '25');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/LoadTorque'], 'Position', [50 290 120 330], 'Value', '0.0');

    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/PosErr'], 'Position', [170 50 210 90], 'Inputs', '+-');
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Kp_pos'], 'Position', [240 50 300 90], 'Gain', '5.0');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/PosInt'], 'Position', [240 110 300 160]);
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/OmegaRef'], 'Position', [330 70 380 140], 'Inputs', '++');

    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/SpeedErr'], 'Position', [450 70 490 110], 'Inputs', '+-');
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Kp_speed'], 'Position', [520 70 560 110], 'Gain', '0.15');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/SpeedInt'], 'Position', [520 130 560 180]);
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/IqRef'], 'Position', [600 90 640 150], 'Inputs', '++');

    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/IdErr'], 'Position', [450 150 490 190], 'Inputs', '+-');
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/IqErr'], 'Position', [450 220 490 260], 'Inputs', '+-');
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Kp_id'], 'Position', [520 150 560 190], 'Gain', '40.0');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/IdInt'], 'Position', [520 210 560 260]);
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Kp_iq'], 'Position', [520 290 560 330], 'Gain', '40.0');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/IqInt'], 'Position', [520 350 560 400]);

    add_block('simulink/Math Operations/Product', [modelName '/Omega_e_x_Lq'], 'Position', [670 100 710 140]);
    add_block('simulink/Math Operations/Product', [modelName '/Feedforward_d'], 'Position', [760 100 800 140]);
    add_block('simulink/Math Operations/Product', [modelName '/Omega_e_x_Ld'], 'Position', [670 200 710 240]);
    add_block('simulink/Math Operations/Product', [modelName '/Feedforward_q1'], 'Position', [760 200 800 240]);
    add_block('simulink/Math Operations/Product', [modelName '/Omega_e_x_psi_d'], 'Position', [670 270 710 310]);
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/CrossCouple_q'], 'Position', [820 205 860 255], 'Inputs', '++');

    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/Vd'], 'Position', [840 100 880 140], 'Inputs', '+++');
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/Vq'], 'Position', [900 200 940 260], 'Inputs', '+-');

    % Motor lookup and dynamics
    add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/MotorLookup'], 'Position', [330 240 450 320]);
    add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/PMSM_Dynamics'], 'Position', [870 20 970 100]);
    set_param([modelName '/MotorLookup'], 'FunctionName', 'motor_lookup');
    set_param([modelName '/MotorLookup'], 'Script', sprintf([ ...
        'function [Ld,Lq,psi_d,psi_q,omega_e] = motor_lookup(Id,Iq,omega_m,temp)\n' ...
        'persistent motor\n' ...
        'if isempty(motor)\n' ...
        '    motor = SurfaceMountedPMSM();\n' ...
        'end\n' ...
        '[Ld,Lq,psi_d,psi_q] = motor.lookup(Id,Iq);\n' ...
        'omega_e = motor.PolePairs * omega_m;\n' ...
        ]));
    set_param([modelName '/PMSM_Dynamics'], 'FunctionName', 'pmsm_dynamics');
    set_param([modelName '/PMSM_Dynamics'], 'Script', sprintf([ ...
        'function [dId,dIq,Te] = pmsm_dynamics(vd,vq,Id,Iq,omega_m,temp,Tl)\n' ...
        'persistent motor\n' ...
        'if isempty(motor)\n' ...
        '    motor = SurfaceMountedPMSM();\n' ...
        'end\n' ...
        '[Ld,Lq,psi_d,psi_q] = motor.lookup(Id,Iq);\n' ...
        'Rs = motor.interpRs(temp);\n' ...
        'omega_e = motor.PolePairs * omega_m;\n' ...
        'dId = (vd - Rs .* Id + omega_e .* Lq .* Iq) ./ Ld;\n' ...
        'dIq = (vq - Rs .* Iq - omega_e .* Ld .* Id - omega_e .* psi_d) ./ Lq;\n' ...
        'Te = 1.5 * motor.PolePairs .* (psi_q .* Iq + (Ld - Lq) .* Id .* Iq);\n' ...
        ]));

    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Id'], 'Position', [1010 20 1050 70]);
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Iq'], 'Position', [1010 100 1050 150]);
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/InvJ'], 'Position', [870 150 910 190], 'Gain', '200');
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/B_gain'], 'Position', [840 150 880 190], 'Gain', '-0.001');
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/OmegaDeriv'], 'Position', [760 150 820 190], 'Inputs', '++-');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Omega_m'], 'Position', [1010 180 1050 230]);
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Theta_m'], 'Position', [1010 260 1050 310]);
    add_block('simulink/Signal Routing/Mux', [modelName '/Mux'], 'Position', [1030 340 1070 420], 'Inputs', '4');
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [1100 340 1180 420]);
    % Connect signals
    add_line(modelName, 'PositionRef/1', 'PosErr/1');
    add_line(modelName, 'PosErr/1', 'Kp_pos/1');
    add_line(modelName, 'PosErr/1', 'PosInt/1');
    add_line(modelName, 'Kp_pos/1', 'OmegaRef/1');
    add_line(modelName, 'PosInt/1', 'OmegaRef/2');

    add_line(modelName, 'OmegaRef/1', 'SpeedErr/1');
    add_line(modelName, 'Omega_m/1', 'SpeedErr/2');
    add_line(modelName, 'SpeedErr/1', 'Kp_speed/1');
    add_line(modelName, 'SpeedErr/1', 'SpeedInt/1');
    add_line(modelName, 'Kp_speed/1', 'IqRef/1');
    add_line(modelName, 'SpeedInt/1', 'IqRef/2');

    add_line(modelName, 'IdRef/1', 'IdErr/1');
    add_line(modelName, 'Id/1', 'IdErr/2');
    add_line(modelName, 'IqRef/1', 'IqErr/1');
    add_line(modelName, 'Iq/1', 'IqErr/2');

    add_line(modelName, 'IdErr/1', 'Kp_id/1');
    add_line(modelName, 'IdErr/1', 'IdInt/1');
    add_line(modelName, 'Kp_id/1', 'Vd/1');
    add_line(modelName, 'IdInt/1', 'Vd/2');

    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/IqPI'], 'Position', [720 170 760 210], 'Inputs', '++');
    add_line(modelName, 'IqErr/1', 'Kp_iq/1');
    add_line(modelName, 'IqErr/1', 'IqInt/1');
    add_line(modelName, 'Kp_iq/1', 'IqPI/1');
    add_line(modelName, 'IqInt/1', 'IqPI/2');
    add_line(modelName, 'IqPI/1', 'Vq/1');

    add_line(modelName, 'Id/1', 'MotorLookup/1');
    add_line(modelName, 'Iq/1', 'MotorLookup/2');
    add_line(modelName, 'Omega_m/1', 'MotorLookup/3');
    add_line(modelName, 'Temperature/1', 'MotorLookup/4');

    add_line(modelName, 'MotorLookup/5', 'Omega_e_x_Lq/1');
    add_line(modelName, 'MotorLookup/2', 'Omega_e_x_Lq/2');
    add_line(modelName, 'MotorLookup/5', 'Omega_e_x_Ld/1');
    add_line(modelName, 'MotorLookup/1', 'Omega_e_x_Ld/2');
    add_line(modelName, 'MotorLookup/5', 'Omega_e_x_psi_d/1');
    add_line(modelName, 'MotorLookup/3', 'Omega_e_x_psi_d/2');
    add_line(modelName, 'Omega_e_x_Lq/1', 'Feedforward_d/1');
    add_line(modelName, 'Iq/1', 'Feedforward_d/2');
    add_line(modelName, 'Omega_e_x_Ld/1', 'Feedforward_q1/1');
    add_line(modelName, 'Id/1', 'Feedforward_q1/2');
    add_line(modelName, 'Feedforward_q1/1', 'CrossCouple_q/1');
    add_line(modelName, 'Omega_e_x_psi_d/1', 'CrossCouple_q/2');
    add_line(modelName, 'CrossCouple_q/1', 'Vq/2');

    add_line(modelName, 'Feedforward_d/1', 'Vd/3');

    add_line(modelName, 'Vd/1', 'PMSM_Dynamics/1');
    add_line(modelName, 'Vq/1', 'PMSM_Dynamics/2');
    add_line(modelName, 'Id/1', 'PMSM_Dynamics/3');
    add_line(modelName, 'Iq/1', 'PMSM_Dynamics/4');
    add_line(modelName, 'Omega_m/1', 'PMSM_Dynamics/5');
    add_line(modelName, 'Temperature/1', 'PMSM_Dynamics/6');
    add_line(modelName, 'LoadTorque/1', 'PMSM_Dynamics/7');

    add_line(modelName, 'PMSM_Dynamics/1', 'Id/1');
    add_line(modelName, 'PMSM_Dynamics/2', 'Iq/1');
    add_line(modelName, 'PMSM_Dynamics/3', 'OmegaDeriv/1');

    add_line(modelName, 'Omega_m/1', 'B_gain/1');
    add_line(modelName, 'B_gain/1', 'OmegaDeriv/2');
    add_line(modelName, 'LoadTorque/1', 'OmegaDeriv/3');
    add_line(modelName, 'OmegaDeriv/1', 'InvJ/1');
    add_line(modelName, 'InvJ/1', 'Omega_m/1');

    add_line(modelName, 'Omega_m/1', 'Theta_m/1');

    add_line(modelName, 'Theta_m/1', 'PosErr/2');
    add_line(modelName, 'Omega_m/1', 'SpeedErr/2');
    add_line(modelName, 'IqRef/1', 'IqErr/1');
    add_line(modelName, 'Id/1', 'IdErr/2');

    add_line(modelName, 'Id/1', 'Mux/1');
    add_line(modelName, 'Iq/1', 'Mux/2');
    add_line(modelName, 'Omega_m/1', 'Mux/3');
    add_line(modelName, 'Theta_m/1', 'Mux/4');
    add_line(modelName, 'Mux/1', 'Scope/1');

    save_system(modelName);
    close_system(modelName, 0);
end
