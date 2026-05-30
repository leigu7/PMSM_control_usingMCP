function generate_surface_mounted_pmsm_foc_model_structured()
% generate_surface_mounted_pmsm_foc_model_structured  Create a structured Simulink FOC model for a surface-mounted PMSM.
% The model explicitly builds separate subsystems for position control, speed control,
% current control, motor lookup, and the motor plant. Block gains and constants use
% workspace variables so parameters can be edited from the MATLAB base workspace.

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

    % Default workspace parameters.
    defaultParams = {
        'PositionRefAmplitude', 2*pi;
        'PositionRefFrequency', 1;
        'IdRef', 0;
        'Temperature', 25;
        'LoadTorque', 0;
        'Kp_pos', 5;
        'Ki_pos', 20;
        'Kp_speed', 0.15;
        'Ki_speed', 1;
        'Kp_id', 40;
        'Ki_id', 200;
        'Kp_iq', 40;
        'Ki_iq', 200;
        'PolePairs', 3;
        'J', 0.01;
        'B', 0.001;
        'Vdc', 600;
    };
    for idx = 1:size(defaultParams, 1)
        varName = defaultParams{idx, 1};
        varValue = defaultParams{idx, 2};
        if evalin('base', ['exist(''' varName ''', ''var'') == 0'])
            assignin('base', varName, varValue);
        end
    end

    % Top-level source and parameter blocks.
    add_block('simulink/Sources/Sine Wave', [modelName '/PositionRef'], 'Position', [30 40 100 80], ...
        'Amplitude', 'PositionRefAmplitude', 'Frequency', 'PositionRefFrequency', 'SampleTime', '0');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/IdRef'], 'Position', [30 120 100 160], 'Value', 'IdRef');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/Temperature'], 'Position', [30 200 100 240], 'Value', 'Temperature');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/LoadTorque'], 'Position', [30 280 100 320], 'Value', 'LoadTorque');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/PolePairs'], 'Position', [30 360 100 400], 'Value', 'PolePairs');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/J'], 'Position', [30 440 100 480], 'Value', 'J');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/B'], 'Position', [30 520 100 560], 'Value', 'B');
    add_block('simulink/Commonly Used Blocks/Constant', [modelName '/Vdc'], 'Position', [30 600 100 640], 'Value', 'Vdc');

    % Top-level subsystem containers.
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Position Controller'], 'Position', [140 30 310 170]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Speed Controller'], 'Position', [140 210 310 330]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Current Controller'], 'Position', [330 30 710 330]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Motor Lookup'], 'Position', [330 360 710 520]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/SVPWM Modulator'], 'Position', [740 360 950 520]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Motor Plant'], 'Position', [980 30 1320 520]);
    add_block('simulink/Commonly Used Blocks/Mux', [modelName '/ScopeMux'], 'Position', [1110 220 1150 300], 'Inputs', '4');
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [1180 220 1240 300]);

    % Position controller internals.
    add_block('simulink/Sources/In1', [modelName '/Position Controller/PositionRef'], 'Position', [30 30 60 50]);
    add_block('simulink/Sources/In1', [modelName '/Position Controller/Theta_m'], 'Position', [30 90 60 110]);
    add_block('simulink/Math Operations/Sum', [modelName '/Position Controller/PosErr'], 'Position', [90 35 120 65], 'Inputs', '+-');
    add_block('simulink/Math Operations/Gain', [modelName '/Position Controller/Kp_pos'], 'Position', [150 20 180 50], 'Gain', 'Kp_pos');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Position Controller/PosInt'], 'Position', [150 80 180 110]);
    add_block('simulink/Math Operations/Sum', [modelName '/Position Controller/Combine'], 'Position', [210 40 240 70], 'Inputs', '++');
    add_block('simulink/Sinks/Out1', [modelName '/Position Controller/Omega_ref'], 'Position', [290 40 320 60]);
    add_line([modelName '/Position Controller'], 'PositionRef/1', 'PosErr/1');
    add_line([modelName '/Position Controller'], 'Theta_m/1', 'PosErr/2');
    add_line([modelName '/Position Controller'], 'PosErr/1', 'Kp_pos/1');
    add_line([modelName '/Position Controller'], 'PosErr/1', 'PosInt/1');
    add_line([modelName '/Position Controller'], 'Kp_pos/1', 'Combine/1');
    add_line([modelName '/Position Controller'], 'PosInt/1', 'Combine/2');
    add_line([modelName '/Position Controller'], 'Combine/1', 'Omega_ref/1');

    % Speed controller internals.
    add_block('simulink/Sources/In1', [modelName '/Speed Controller/Omega_ref'], 'Position', [30 30 60 50]);
    add_block('simulink/Sources/In1', [modelName '/Speed Controller/Omega_m'], 'Position', [30 90 60 110]);
    add_block('simulink/Math Operations/Sum', [modelName '/Speed Controller/SpeedErr'], 'Position', [90 35 120 65], 'Inputs', '+-');
    add_block('simulink/Math Operations/Gain', [modelName '/Speed Controller/Kp_speed'], 'Position', [150 20 180 50], 'Gain', 'Kp_speed');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Speed Controller/SpeedInt'], 'Position', [150 80 180 110]);
    add_block('simulink/Math Operations/Sum', [modelName '/Speed Controller/Combine'], 'Position', [210 40 240 70], 'Inputs', '++');
    add_block('simulink/Sinks/Out1', [modelName '/Speed Controller/IqRef'], 'Position', [290 40 320 60]);
    add_line([modelName '/Speed Controller'], 'Omega_ref/1', 'SpeedErr/1');
    add_line([modelName '/Speed Controller'], 'Omega_m/1', 'SpeedErr/2');
    add_line([modelName '/Speed Controller'], 'SpeedErr/1', 'Kp_speed/1');
    add_line([modelName '/Speed Controller'], 'SpeedErr/1', 'SpeedInt/1');
    add_line([modelName '/Speed Controller'], 'Kp_speed/1', 'Combine/1');
    add_line([modelName '/Speed Controller'], 'SpeedInt/1', 'Combine/2');
    add_line([modelName '/Speed Controller'], 'Combine/1', 'IqRef/1');

    % Current controller internals.
    add_block('simulink/Sources/In1', [modelName '/Current Controller/IdRef'], 'Position', [30 30 60 50]);
    add_block('simulink/Sources/In1', [modelName '/Current Controller/IqRef'], 'Position', [30 80 60 100]);
    add_block('simulink/Sources/In1', [modelName '/Current Controller/Id'], 'Position', [30 130 60 150]);
    add_block('simulink/Sources/In1', [modelName '/Current Controller/Iq'], 'Position', [30 180 60 200]);
    add_block('simulink/Sources/In1', [modelName '/Current Controller/omega_e'], 'Position', [30 230 60 250]);
    add_block('simulink/Sources/In1', [modelName '/Current Controller/Ld'], 'Position', [30 280 60 300]);
    add_block('simulink/Sources/In1', [modelName '/Current Controller/Lq'], 'Position', [30 330 60 350]);
    add_block('simulink/Sources/In1', [modelName '/Current Controller/psi_d'], 'Position', [30 380 60 400]);

    add_block('simulink/Math Operations/Sum', [modelName '/Current Controller/IdErr'], 'Position', [100 30 130 60], 'Inputs', '+-');
    add_block('simulink/Math Operations/Sum', [modelName '/Current Controller/IqErr'], 'Position', [100 80 130 110], 'Inputs', '+-');
    add_block('simulink/Math Operations/Gain', [modelName '/Current Controller/Kp_id'], 'Position', [170 30 200 60], 'Gain', 'Kp_id');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Current Controller/IdInt'], 'Position', [170 80 200 110]);
    add_block('simulink/Math Operations/Gain', [modelName '/Current Controller/Kp_iq'], 'Position', [170 130 200 160], 'Gain', 'Kp_iq');
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Current Controller/IqInt'], 'Position', [170 180 200 210]);

    add_block('simulink/Math Operations/Product', [modelName '/Current Controller/omega_e_x_Lq'], 'Position', [240 230 270 260]);
    add_block('simulink/Math Operations/Product', [modelName '/Current Controller/vd_ff'], 'Position', [300 230 330 260]);
    add_block('simulink/Math Operations/Product', [modelName '/Current Controller/omega_e_x_Ld'], 'Position', [240 280 270 310]);
    add_block('simulink/Math Operations/Product', [modelName '/Current Controller/omega_e_x_Ld_Id'], 'Position', [300 280 330 310]);
    add_block('simulink/Math Operations/Product', [modelName '/Current Controller/omega_e_x_psi_d'], 'Position', [240 330 270 360]);
    add_block('simulink/Math Operations/Sum', [modelName '/Current Controller/Vd_sum'], 'Position', [370 230 400 260], 'Inputs', '+++');
    add_block('simulink/Math Operations/Sum', [modelName '/Current Controller/Vq_sum'], 'Position', [370 300 400 340], 'Inputs', '++--');
    add_block('simulink/Sinks/Out1', [modelName '/Current Controller/Vd'], 'Position', [470 230 500 250]);
    add_block('simulink/Sinks/Out1', [modelName '/Current Controller/Vq'], 'Position', [470 300 500 320]);

    add_line([modelName '/Current Controller'], 'IdRef/1', 'IdErr/1');
    add_line([modelName '/Current Controller'], 'Id/1', 'IdErr/2');
    add_line([modelName '/Current Controller'], 'IqRef/1', 'IqErr/1');
    add_line([modelName '/Current Controller'], 'Iq/1', 'IqErr/2');
    add_line([modelName '/Current Controller'], 'IdErr/1', 'Kp_id/1');
    add_line([modelName '/Current Controller'], 'IdErr/1', 'IdInt/1');
    add_line([modelName '/Current Controller'], 'Kp_id/1', 'Vd_sum/1');
    add_line([modelName '/Current Controller'], 'IdInt/1', 'Vd_sum/2');
    add_line([modelName '/Current Controller'], 'omega_e_x_Lq/1', 'vd_ff/1');
    add_line([modelName '/Current Controller'], 'Iq/1', 'vd_ff/2');
    add_line([modelName '/Current Controller'], 'vd_ff/1', 'Vd_sum/3');
    add_line([modelName '/Current Controller'], 'Vd_sum/1', 'Vd/1');
    add_line([modelName '/Current Controller'], 'IqErr/1', 'Kp_iq/1');
    add_line([modelName '/Current Controller'], 'IqErr/1', 'IqInt/1');
    add_line([modelName '/Current Controller'], 'Kp_iq/1', 'Vq_sum/1');
    add_line([modelName '/Current Controller'], 'IqInt/1', 'Vq_sum/2');
    add_line([modelName '/Current Controller'], 'omega_e_x_Ld/1', 'omega_e_x_Ld_Id/1');
    add_line([modelName '/Current Controller'], 'Id/1', 'omega_e_x_Ld_Id/2');
    add_line([modelName '/Current Controller'], 'omega_e_x_Ld_Id/1', 'Vq_sum/3');
    add_line([modelName '/Current Controller'], 'omega_e_x_psi_d/1', 'Vq_sum/4');
    add_line([modelName '/Current Controller'], 'Vq_sum/1', 'Vq/1');
    add_line([modelName '/Current Controller'], 'omega_e/1', 'omega_e_x_Lq/1');
    add_line([modelName '/Current Controller'], 'Lq/1', 'omega_e_x_Lq/2');
    add_line([modelName '/Current Controller'], 'omega_e/1', 'omega_e_x_Ld/1');
    add_line([modelName '/Current Controller'], 'Ld/1', 'omega_e_x_Ld/2');
    add_line([modelName '/Current Controller'], 'omega_e/1', 'omega_e_x_psi_d/1');
    add_line([modelName '/Current Controller'], 'psi_d/1', 'omega_e_x_psi_d/2');

    % Motor lookup internals.
    add_block('simulink/Sources/In1', [modelName '/Motor Lookup/Id'], 'Position', [30 30 60 50]);
    add_block('simulink/Sources/In1', [modelName '/Motor Lookup/Iq'], 'Position', [30 80 60 100]);
    add_block('simulink/Sources/In1', [modelName '/Motor Lookup/Omega_m'], 'Position', [30 130 60 150]);
    add_block('simulink/Sources/In1', [modelName '/Motor Lookup/Temperature'], 'Position', [30 180 60 200]);
    add_block('simulink/Sources/In1', [modelName '/Motor Lookup/PolePairs'], 'Position', [30 230 60 250]);
    add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Motor Lookup/MotorLookupFcn'], 'Position', [110 80 220 190]);
    set_param([modelName '/Motor Lookup/MotorLookupFcn'], 'FunctionName', 'motor_lookup');
    set_param([modelName '/Motor Lookup/MotorLookupFcn'], 'Script', sprintf([ ...
        'function [Ld,Lq,psi_d,psi_q,omega_e] = motor_lookup(Id,Iq,omega_m,temp,PolePairs)\n' ...
        'persistent motor\n' ...
        'if isempty(motor)\n' ...
        '    motor = SurfaceMountedPMSM();\n' ...
        'end\n' ...
        '[Ld,Lq,psi_d,psi_q] = motor.lookup(Id,Iq);\n' ...
        'omega_e = PolePairs .* omega_m;\n' ...
        ]));
    add_block('simulink/Sinks/Out1', [modelName '/Motor Lookup/Ld'], 'Position', [300 30 330 50]);
    add_block('simulink/Sinks/Out1', [modelName '/Motor Lookup/Lq'], 'Position', [300 80 330 100]);
    add_block('simulink/Sinks/Out1', [modelName '/Motor Lookup/psi_d'], 'Position', [300 130 330 150]);
    add_block('simulink/Sinks/Out1', [modelName '/Motor Lookup/psi_q'], 'Position', [300 180 330 200]);
    add_block('simulink/Sinks/Out1', [modelName '/Motor Lookup/omega_e'], 'Position', [300 230 330 250]);
    add_line([modelName '/Motor Lookup'], 'Id/1', 'MotorLookupFcn/1');
    add_line([modelName '/Motor Lookup'], 'Iq/1', 'MotorLookupFcn/2');
    add_line([modelName '/Motor Lookup'], 'Omega_m/1', 'MotorLookupFcn/3');
    add_line([modelName '/Motor Lookup'], 'Temperature/1', 'MotorLookupFcn/4');
    add_line([modelName '/Motor Lookup'], 'PolePairs/1', 'MotorLookupFcn/5');
    add_line([modelName '/Motor Lookup'], 'MotorLookupFcn/1', 'Ld/1');
    add_line([modelName '/Motor Lookup'], 'MotorLookupFcn/2', 'Lq/1');
    add_line([modelName '/Motor Lookup'], 'MotorLookupFcn/3', 'psi_d/1');
    add_line([modelName '/Motor Lookup'], 'MotorLookupFcn/4', 'psi_q/1');
    add_line([modelName '/Motor Lookup'], 'MotorLookupFcn/5', 'omega_e/1');

    % SVPWM modulator internals.
    add_block('simulink/Sources/In1', [modelName '/SVPWM Modulator/Vd'], 'Position', [30 30 60 50]);
    add_block('simulink/Sources/In1', [modelName '/SVPWM Modulator/Vq'], 'Position', [30 80 60 100]);
    add_block('simulink/Sources/In1', [modelName '/SVPWM Modulator/Theta_m'], 'Position', [30 130 60 150]);
    add_block('simulink/Sources/In1', [modelName '/SVPWM Modulator/PolePairs'], 'Position', [30 180 60 200]);
    add_block('simulink/Sources/In1', [modelName '/SVPWM Modulator/Vdc'], 'Position', [30 230 60 250]);
    add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/SVPWM Modulator/SVPWMFcn'], 'Position', [110 80 220 220]);
    set_param([modelName '/SVPWM Modulator/SVPWMFcn'], 'FunctionName', 'svpwm_modulator');
    set_param([modelName '/SVPWM Modulator/SVPWMFcn'], 'Script', sprintf([ ...
        'function [Vd_out,Vq_out,Va,Vb,Vc] = svpwm_modulator(Vd,Vq,theta_m,PolePairs,Vdc)\n' ...
        'theta_e = PolePairs .* theta_m;\n' ...
        'Valpha = Vd .* cos(theta_e) - Vq .* sin(theta_e);\n' ...
        'Vbeta = Vd .* sin(theta_e) + Vq .* cos(theta_e);\n' ...
        'Vref = sqrt(Valpha.^2 + Vbeta.^2);\n' ...
        'Vmax = Vdc ./ sqrt(3);\n' ...
        'scale = ones(size(Vref));\n' ...
        'idx = Vref > Vmax;\n' ...
        'scale(idx) = Vmax(idx) ./ Vref(idx);\n' ...
        'Valpha = Valpha .* scale;\n' ...
        'Vbeta = Vbeta .* scale;\n' ...
        'Vd_out =  Valpha .* cos(theta_e) + Vbeta .* sin(theta_e);\n' ...
        'Vq_out = -Valpha .* sin(theta_e) + Vbeta .* cos(theta_e);\n' ...
        'Va = Valpha;\n' ...
        'Vb = -0.5 .* Valpha + sqrt(3)/2 .* Vbeta;\n' ...
        'Vc = -0.5 .* Valpha - sqrt(3)/2 .* Vbeta;\n' ...
        ]));
    add_block('simulink/Sinks/Out1', [modelName '/SVPWM Modulator/Vd_out'], 'Position', [300 30 330 50]);
    add_block('simulink/Sinks/Out1', [modelName '/SVPWM Modulator/Vq_out'], 'Position', [300 80 330 100]);
    add_block('simulink/Sinks/Out1', [modelName '/SVPWM Modulator/Va'], 'Position', [300 130 330 150]);
    add_block('simulink/Sinks/Out1', [modelName '/SVPWM Modulator/Vb'], 'Position', [300 180 330 200]);
    add_block('simulink/Sinks/Out1', [modelName '/SVPWM Modulator/Vc'], 'Position', [300 230 330 250]);
    add_line([modelName '/SVPWM Modulator'], 'Vd/1', 'SVPWMFcn/1');
    add_line([modelName '/SVPWM Modulator'], 'Vq/1', 'SVPWMFcn/2');
    add_line([modelName '/SVPWM Modulator'], 'Theta_m/1', 'SVPWMFcn/3');
    add_line([modelName '/SVPWM Modulator'], 'PolePairs/1', 'SVPWMFcn/4');
    add_line([modelName '/SVPWM Modulator'], 'Vdc/1', 'SVPWMFcn/5');
    add_line([modelName '/SVPWM Modulator'], 'SVPWMFcn/1', 'Vd_out/1');
    add_line([modelName '/SVPWM Modulator'], 'SVPWMFcn/2', 'Vq_out/1');
    add_line([modelName '/SVPWM Modulator'], 'SVPWMFcn/3', 'Va/1');
    add_line([modelName '/SVPWM Modulator'], 'SVPWMFcn/4', 'Vb/1');
    add_line([modelName '/SVPWM Modulator'], 'SVPWMFcn/5', 'Vc/1');

    % Motor plant internals.
    add_block('simulink/Sources/In1', [modelName '/Motor Plant/Vd'], 'Position', [30 30 60 50]);
    add_block('simulink/Sources/In1', [modelName '/Motor Plant/Vq'], 'Position', [30 90 60 110]);
    add_block('simulink/Sources/In1', [modelName '/Motor Plant/Temperature'], 'Position', [30 150 60 170]);
    add_block('simulink/Sources/In1', [modelName '/Motor Plant/LoadTorque'], 'Position', [30 210 60 230]);
    add_block('simulink/Sources/In1', [modelName '/Motor Plant/PolePairs'], 'Position', [30 270 60 290]);
    add_block('simulink/Sources/In1', [modelName '/Motor Plant/J'], 'Position', [30 330 60 350]);
    add_block('simulink/Sources/In1', [modelName '/Motor Plant/B'], 'Position', [30 390 60 410]);
    add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Motor Plant/MotorDynamics'], 'Position', [130 90 240 210]);
    set_param([modelName '/Motor Plant/MotorDynamics'], 'FunctionName', 'pmsm_dynamics');
    set_param([modelName '/Motor Plant/MotorDynamics'], 'Script', sprintf([ ...
        'function [dId,dIq,Te] = pmsm_dynamics(vd,vq,Id,Iq,omega_m,temp,Tl,PolePairs)\n' ...
        'persistent motor\n' ...
        'if isempty(motor)\n' ...
        '    motor = SurfaceMountedPMSM();\n' ...
        'end\n' ...
        '[Ld,Lq,psi_d,psi_q] = motor.lookup(Id,Iq);\n' ...
        'Rs = motor.interpRs(temp);\n' ...
        'omega_e = PolePairs .* omega_m;\n' ...
        'dId = (vd - Rs .* Id + omega_e .* Lq .* Iq) ./ Ld;\n' ...
        'dIq = (vq - Rs .* Iq - omega_e .* Ld .* Id - omega_e .* psi_d) ./ Lq;\n' ...
        'Te = 1.5 * PolePairs .* (psi_q .* Iq + (Ld - Lq) .* Id .* Iq);\n' ...
        ]));
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Motor Plant/Id'], 'Position', [350 30 390 70]);
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Motor Plant/Iq'], 'Position', [350 110 390 150]);
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Motor Plant/Omega_m'], 'Position', [350 190 390 230]);
    add_block('simulink/Commonly Used Blocks/Integrator', [modelName '/Motor Plant/Theta_m'], 'Position', [350 270 390 310]);
    add_block('simulink/Math Operations/Gain', [modelName '/Motor Plant/B_gain'], 'Position', [420 190 450 220], 'Gain', '-B');
    add_block('simulink/Math Operations/Sum', [modelName '/Motor Plant/OmegaNet'], 'Position', [470 190 500 220], 'Inputs', '+--');
    add_block('simulink/Math Operations/Gain', [modelName '/Motor Plant/InvJ'], 'Position', [530 190 560 220], 'Gain', '1./J');
    add_block('simulink/Sinks/Out1', [modelName '/Motor Plant/Id_out'], 'Position', [450 30 480 50]);
    add_block('simulink/Sinks/Out1', [modelName '/Motor Plant/Iq_out'], 'Position', [450 110 480 130]);
    add_block('simulink/Sinks/Out1', [modelName '/Motor Plant/Omega_m_out'], 'Position', [450 190 480 210]);
    add_block('simulink/Sinks/Out1', [modelName '/Motor Plant/Theta_m_out'], 'Position', [450 270 480 290]);

    add_line([modelName '/Motor Plant'], 'Vd/1', 'MotorDynamics/1');
    add_line([modelName '/Motor Plant'], 'Vq/1', 'MotorDynamics/2');
    add_line([modelName '/Motor Plant'], 'Id/1', 'MotorDynamics/3');
    add_line([modelName '/Motor Plant'], 'Iq/1', 'MotorDynamics/4');
    add_line([modelName '/Motor Plant'], 'Omega_m/1', 'MotorDynamics/5');
    add_line([modelName '/Motor Plant'], 'Temperature/1', 'MotorDynamics/6');
    add_line([modelName '/Motor Plant'], 'LoadTorque/1', 'MotorDynamics/7');
    add_line([modelName '/Motor Plant'], 'PolePairs/1', 'MotorDynamics/8');
    add_line([modelName '/Motor Plant'], 'MotorDynamics/1', 'Id/1');
    add_line([modelName '/Motor Plant'], 'MotorDynamics/2', 'Iq/1');
    add_line([modelName '/Motor Plant'], 'MotorDynamics/3', 'OmegaNet/1');
    add_line([modelName '/Motor Plant'], 'Omega_m/1', 'B_gain/1');
    add_line([modelName '/Motor Plant'], 'B_gain/1', 'OmegaNet/2');
    add_line([modelName '/Motor Plant'], 'LoadTorque/1', 'OmegaNet/3');
    add_line([modelName '/Motor Plant'], 'OmegaNet/1', 'InvJ/1');
    add_line([modelName '/Motor Plant'], 'InvJ/1', 'Omega_m/1');
    add_line([modelName '/Motor Plant'], 'Omega_m/1', 'Theta_m/1');
    add_line([modelName '/Motor Plant'], 'Id/1', 'Id_out/1');
    add_line([modelName '/Motor Plant'], 'Iq/1', 'Iq_out/1');
    add_line([modelName '/Motor Plant'], 'Omega_m/1', 'Omega_m_out/1');
    add_line([modelName '/Motor Plant'], 'Theta_m/1', 'Theta_m_out/1');

    % Top-level signal routing.
    add_line(modelName, 'PositionRef/1', 'Position Controller/1');
    add_line(modelName, 'Motor Plant/Theta_m_out', 'Position Controller/2');
    add_line(modelName, 'Position Controller/1', 'Speed Controller/1');
    add_line(modelName, 'Motor Plant/Omega_m_out', 'Speed Controller/2');
    add_line(modelName, 'Speed Controller/1', 'Current Controller/2');
    add_line(modelName, 'IdRef/1', 'Current Controller/1');
    add_line(modelName, 'Motor Plant/Id_out', 'Current Controller/3');
    add_line(modelName, 'Motor Plant/Iq_out', 'Current Controller/4');
    add_line(modelName, 'Motor Lookup/omega_e', 'Current Controller/5');
    add_line(modelName, 'Motor Lookup/Ld', 'Current Controller/6');
    add_line(modelName, 'Motor Lookup/Lq', 'Current Controller/7');
    add_line(modelName, 'Motor Lookup/psi_d', 'Current Controller/8');
    add_line(modelName, 'Current Controller/Vd', 'SVPWM Modulator/1');
    add_line(modelName, 'Current Controller/Vq', 'SVPWM Modulator/2');
    add_line(modelName, 'Motor Plant/Theta_m_out', 'SVPWM Modulator/3');
    add_line(modelName, 'PolePairs/1', 'SVPWM Modulator/4');
    add_line(modelName, 'Vdc/1', 'SVPWM Modulator/5');
    add_line(modelName, 'SVPWM Modulator/1', 'Motor Plant/1');
    add_line(modelName, 'SVPWM Modulator/2', 'Motor Plant/2');
    add_line(modelName, 'Temperature/1', 'Motor Plant/3');
    add_line(modelName, 'LoadTorque/1', 'Motor Plant/4');
    add_line(modelName, 'PolePairs/1', 'Motor Plant/5');
    add_line(modelName, 'J/1', 'Motor Plant/6');
    add_line(modelName, 'B/1', 'Motor Plant/7');
    add_line(modelName, 'Motor Plant/Id_out', 'Motor Lookup/1');
    add_line(modelName, 'Motor Plant/Iq_out', 'Motor Lookup/2');
    add_line(modelName, 'Motor Plant/Omega_m_out', 'Motor Lookup/3');
    add_line(modelName, 'Temperature/1', 'Motor Lookup/4');
    add_line(modelName, 'PolePairs/1', 'Motor Lookup/5');
    add_line(modelName, 'Motor Plant/Id_out', 'ScopeMux/1');
    add_line(modelName, 'Motor Plant/Iq_out', 'ScopeMux/2');
    add_line(modelName, 'Motor Plant/Omega_m_out', 'ScopeMux/3');
    add_line(modelName, 'Motor Plant/Theta_m_out', 'ScopeMux/4');
    add_line(modelName, 'ScopeMux/1', 'Scope/1');

    save_system(modelName);
    close_system(modelName, 0);
end