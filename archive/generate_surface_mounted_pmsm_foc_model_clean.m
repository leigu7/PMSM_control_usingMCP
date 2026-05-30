function generate_surface_mounted_pmsm_foc_model_clean()
% generate_surface_mounted_pmsm_foc_model_clean  Create a clean Simulink FOC model for a surface-mounted PMSM.
% The model includes: position feedback, speed control, inner dq current loops,
% temperature-dependent Rs, and 2D lookup tables for Ld/Lq/flux.

    modelName = 'surface_mounted_pmsm_foc';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    if exist([modelName '.slx'], 'file')
        delete([modelName '.slx']);
    end

    new_system(modelName, 'Model');
    open_system(modelName);

    set_param(modelName, 'Solver', 'ode45', 'StopTime', '1', 'MaxStep', '1e-4', ...
        'SaveOutput', 'off', 'SaveState', 'off', 'SignalLogging', 'off');

    % Source blocks
    add_block('simulink/Sources/Sine Wave', [modelName '/PositionRef'], 'Position', [50 50 120 90], ...
        'Amplitude', '2*pi', 'Frequency', '1', 'SampleTime', '0');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/IdRef'], 'Position', [50 150 120 190], 'Value', '0');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/Temperature'], 'Position', [50 210 120 250], 'Value', '25');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/LoadTorque'], 'Position', [50 270 120 310], 'Value', '0');

    % Position and speed controller
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/PosErr'], 'Position', [180 50 220 90], 'Inputs', '+-');
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Kp_pos'], 'Position', [250 50 300 90], 'Gain', '5');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/PosInt'], 'Position', [250 110 300 150]);
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/OmegaRef'], 'Position', [330 70 380 130], 'Inputs', '++');

    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/SpeedErr'], 'Position', [430 70 470 110], 'Inputs', '+-');
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Kp_speed'], 'Position', [500 70 540 110], 'Gain', '0.15');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/SpeedInt'], 'Position', [500 130 540 170]);
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/IqRef'], 'Position', [580 90 620 150], 'Inputs', '++');

    % Current controller
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/IdErr'], 'Position', [430 160 470 200], 'Inputs', '+-');
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/IqErr'], 'Position', [430 220 470 260], 'Inputs', '+-');
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Kp_id'], 'Position', [500 160 540 200], 'Gain', '40');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/IdInt'], 'Position', [500 210 540 250]);
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Kp_iq'], 'Position', [500 260 540 300], 'Gain', '40');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/IqInt'], 'Position', [500 310 540 350]);

    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/Vd_sum'], 'Position', [620 150 660 210], 'Inputs', '+++');

    add_block('simulink/Math Operations/Product', [modelName '/omega_e_x_Lq'], 'Position', [680 160 720 200]);
    add_block('simulink/Math Operations/Product', [modelName '/vd_ff'], 'Position', [740 160 780 200]);
    add_block('simulink/Math Operations/Product', [modelName '/omega_e_x_Ld'], 'Position', [680 260 720 300]);
    add_block('simulink/Math Operations/Product', [modelName '/omega_e_x_psi_d'], 'Position', [680 320 720 360]);
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/minus1'], 'Position', [740 320 780 360], 'Inputs', '+-');

    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/Vq_ff_sum'], 'Position', [800 250 840 310], 'Inputs', '++--');

    % Motor model blocks
    add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/MotorLookup'], 'Position', [350 340 470 420]);
    add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/PMSM_Dynamics'], 'Position', [860 150 960 240]);

    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Id'], 'Position', [980 50 1020 90]);
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Iq'], 'Position', [980 120 1020 160]);
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Omega_m'], 'Position', [980 190 1020 230]);
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Theta_m'], 'Position', [980 270 1020 310]);

    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/B_gain'], 'Position', [840 150 880 190], 'Gain', '-0.001');
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/InvJ'], 'Position', [840 110 880 150], 'Gain', '200');
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/OmegaDot'], 'Position', [760 130 800 180], 'Inputs', '++-');

    add_block('simulink/Commonly Used Blocks/Mux', [modelName '/ScopeMux'], 'Position', [1040 330 1080 430], 'Inputs', '4');
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [1100 330 1180 430]);

    % Output routing
    add_block('simulink/Signal Routing/Goto', [modelName '/GotoTheta'], 'Position', [1030 270 1070 300], 'Tag', 'Theta');
    add_block('simulink/Signal Routing/From', [modelName '/FromTheta'], 'Position', [170 60 210 90], 'GotoTag', 'Theta');

    % Connect signals
    add_line(modelName, 'PositionRef/1', 'PosErr/1');
    add_line(modelName, 'FromTheta/1', 'PosErr/2');

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
    add_line(modelName, 'Kp_id/1', 'Vd_sum/1');
    add_line(modelName, 'IdInt/1', 'Vd_sum/2');

    add_line(modelName, 'IqErr/1', 'Kp_iq/1');
    add_line(modelName, 'IqErr/1', 'IqInt/1');
    add_line(modelName, 'Kp_iq/1', 'Vq_ff_sum/1');
    add_line(modelName, 'IqInt/1', 'Vq_ff_sum/2');

    add_line(modelName, 'omega_e_x_Lq/1', 'vd_ff/1');
    add_line(modelName, 'Iq/1', 'vd_ff/2');
    add_line(modelName, 'vd_ff/1', 'Vd_sum/3');

    add_line(modelName, 'omega_e_x_Ld/1', 'minus1/1');
    add_line(modelName, 'Id/1', 'minus1/2');
    add_line(modelName, 'minus1/1', 'Vq_ff_sum/3');
    add_line(modelName, 'omega_e_x_psi_d/1', 'Vq_ff_sum/4');

    add_line(modelName, 'Vd_sum/1', 'PMSM_Dynamics/1');
    add_line(modelName, 'Vq_ff_sum/1', 'PMSM_Dynamics/2');
    add_line(modelName, 'Id/1', 'PMSM_Dynamics/3');
    add_line(modelName, 'Iq/1', 'PMSM_Dynamics/4');
    add_line(modelName, 'Omega_m/1', 'PMSM_Dynamics/5');
    add_line(modelName, 'Temperature/1', 'PMSM_Dynamics/6');
    add_line(modelName, 'LoadTorque/1', 'PMSM_Dynamics/7');

    add_line(modelName, 'PMSM_Dynamics/1', 'Id/1');
    add_line(modelName, 'PMSM_Dynamics/2', 'Iq/1');
    add_line(modelName, 'PMSM_Dynamics/3', 'OmegaDot/1');

    add_line(modelName, 'Omega_m/1', 'OmegaDot/2');
    add_line(modelName, 'LoadTorque/1', 'OmegaDot/3');
    add_line(modelName, 'OmegaDot/1', 'B_gain/1');
    add_line(modelName, 'B_gain/1', 'InvJ/1');
    add_line(modelName, 'InvJ/1', 'Omega_m/1');

    add_line(modelName, 'Omega_m/1', 'Theta_m/1');
    add_line(modelName, 'Theta_m/1', 'GotoTheta/1');
    add_line(modelName, 'GotoTheta/1', 'ScopeMux/4');

    add_line(modelName, 'Id/1', 'ScopeMux/1');
    add_line(modelName, 'Iq/1', 'ScopeMux/2');
    add_line(modelName, 'Omega_m/1', 'ScopeMux/3');
    add_line(modelName, 'ScopeMux/1', 'Scope/1');

    % Motor lookup wiring
    add_line(modelName, 'Id/1', 'MotorLookup/1');
    add_line(modelName, 'Iq/1', 'MotorLookup/2');
    add_line(modelName, 'Omega_m/1', 'MotorLookup/3');
    add_line(modelName, 'Temperature/1', 'MotorLookup/4');

    add_line(modelName, 'MotorLookup/1', 'omega_e_x_Ld/2');
    add_line(modelName, 'MotorLookup/2', 'omega_e_x_Lq/2');
    add_line(modelName, 'MotorLookup/5', 'omega_e_x_Lq/1');
    add_line(modelName, 'MotorLookup/5', 'omega_e_x_Ld/1');
    add_line(modelName, 'MotorLookup/3', 'omega_e_x_psi_d/2');
    add_line(modelName, 'MotorLookup/5', 'omega_e_x_psi_d/1');

    % Set MATLAB function scripts
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

    save_system(modelName);
    close_system(modelName, 0);
end
