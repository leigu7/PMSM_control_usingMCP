% Comprehensive model inspection script
clear; clc;

model_name = 'surface_mounted_pmsm_foc';
load_system(model_name);

fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║            MODEL STRUCTURE INSPECTION REPORT                  ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

% 1. Top-level block count
fprintf('1️⃣  TOP-LEVEL BLOCKS:\n');
all_blocks = find_system(model_name, 'SearchDepth', 1);
subsystems = find_system(model_name, 'SearchDepth', 1, 'BlockType', 'SubSystem');
constants = find_system(model_name, 'SearchDepth', 1, 'BlockType', 'Constant');
sine = find_system(model_name, 'SearchDepth', 1, 'BlockType', 'Sine');
mux = find_system(model_name, 'SearchDepth', 1, 'BlockType', 'Mux');
scope = find_system(model_name, 'SearchDepth', 1, 'BlockType', 'Scope');

fprintf('   Total blocks: %d\n', length(all_blocks)-1);  % -1 for model itself
fprintf('   Subsystems: %d\n', length(subsystems));
fprintf('   Constants: %d\n', length(constants));
fprintf('   Sine Wave: %d\n', length(sine));
fprintf('   Mux: %d\n', length(mux));
fprintf('   Scope: %d\n', length(scope));

fprintf('\n   Subsystems found:\n');
for i = 1:length(subsystems)
    [~, name] = fileparts(subsystems{i});
    fprintf('      ✓ %s\n', name);
end

% 2. Check signal connectivity
fprintf('\n2️⃣  SIGNAL CONNECTIVITY:\n');
try
    % Get all lines at top level
    lines = find_system(model_name, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
    fprintf('   Lines at top level: %d\n', length(lines));
    
    % Check for unconnected ports
    fprintf('\n   Checking for unconnected ports...\n');
    unconnected_ports = [];
    blocks_to_check = find_system(model_name, 'SearchDepth', 1);
    
    unconnected_count = 0;
    for i = 2:length(blocks_to_check)  % Skip model root
        try
            block = blocks_to_check{i};
            in_ports = get_param(block, 'InputPorts');
            out_ports = get_param(block, 'OutputPorts');
            
            % Check each port
            for p = 1:length(in_ports)
                try
                    port_h = get_param(block, ['InPort' num2str(p)]);
                    line_h = get_param(port_h, 'Line');
                    if line_h == -1
                        [~, blk_name] = fileparts(block);
                        fprintf('      ⚠️  UNCONNECTED: %s (input %d)\n', blk_name, p);
                        unconnected_count = unconnected_count + 1;
                    end
                catch
                end
            end
        catch
        end
    end
    
    if unconnected_count == 0
        fprintf('      ✓ All ports appear connected\n');
    else
        fprintf('      ⚠️  %d unconnected ports found!\n', unconnected_count);
    end
    
catch ME
    fprintf('   Error checking connectivity: %s\n', ME.message);
end

% 3. Check subsystem internal structure
fprintf('\n3️⃣  SUBSYSTEM INTERNALS:\n');
fprintf('   Checking Position Controller...\n');
try
    pos_ctrl = find_system(model_name, 'SearchDepth', 1, 'Name', 'Position Controller');
    if ~isempty(pos_ctrl)
        pos_blocks = find_system(pos_ctrl{1}, 'SearchDepth', 1);
        fprintf('      Blocks: %d\n', length(pos_blocks)-1);
        % List block types
        for i = 2:min(length(pos_blocks), 10)
            block_type = get_param(pos_blocks{i}, 'BlockType');
            [~, block_name] = fileparts(pos_blocks{i});
            fprintf('         - %s (%s)\n', block_name, block_type);
        end
    end
catch ME
    fprintf('      Error: %s\n', ME.message);
end

fprintf('\n   Checking Motor Plant...\n');
try
    motor = find_system(model_name, 'SearchDepth', 1, 'Name', 'Motor Plant');
    if ~isempty(motor)
        motor_blocks = find_system(motor{1}, 'SearchDepth', 1);
        fprintf('      Blocks: %d\n', length(motor_blocks)-1);
        % Look for integrators and outputs
        integrators = find_system(motor{1}, 'SearchDepth', 1, 'BlockType', 'Integrator');
        outputs = find_system(motor{1}, 'SearchDepth', 1, 'BlockType', 'Outport');
        fprintf('      Integrators: %d\n', length(integrators));
        fprintf('      Output ports: %d\n', length(outputs));
    end
catch ME
    fprintf('      Error: %s\n', ME.message);
end

% 4. Check model solver settings
fprintf('\n4️⃣  MODEL CONFIGURATION:\n');
solver = get_param(model_name, 'Solver');
stop_time = get_param(model_name, 'StopTime');
max_step = get_param(model_name, 'MaxStep');
fprintf('   Solver: %s\n', solver);
fprintf('   StopTime: %s\n', stop_time);
fprintf('   MaxStep: %s\n', max_step);

% 5. Try to simulate a short test run
fprintf('\n5️⃣  SIMULATION TEST:\n');
try
    fprintf('   Attempting short simulation (0.01s)...\n');
    set_param(model_name, 'StopTime', '0.01');
    set_param(model_name, 'MaxStep', '1e-4');
    
    % Set minimal parameters to avoid errors
    assignin('base', 'Kp_pos', 5);
    assignin('base', 'Ki_pos', 20);
    assignin('base', 'Kp_speed', 0.15);
    assignin('base', 'Ki_speed', 1);
    assignin('base', 'Kp_id', 40);
    assignin('base', 'Ki_id', 200);
    assignin('base', 'Kp_iq', 40);
    assignin('base', 'Ki_iq', 200);
    assignin('base', 'PolePairs', 3);
    assignin('base', 'J', 0.01);
    assignin('base', 'B', 0.001);
    assignin('base', 'Vdc', 600);
    assignin('base', 'PositionRefAmplitude', 2*pi);
    assignin('base', 'PositionRefFrequency', 1);
    assignin('base', 'IdRef', 0);
    assignin('base', 'Temperature', 25);
    assignin('base', 'LoadTorque', 0);
    
    sim_output = sim(model_name, 'StopTime', '0.01');
    fprintf('   ✓ Simulation completed successfully!\n');
    fprintf('   Output variables: %s\n', sprintf('%s ', sim_output.who{:}));
    
    % Restore original settings
    set_param(model_name, 'StopTime', '1');
    set_param(model_name, 'MaxStep', '1e-4');
    
catch ME
    fprintf('   ⚠️  Simulation ERROR: %s\n', ME.message);
    set_param(model_name, 'StopTime', '1');
end

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                    END OF INSPECTION                          ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n');

close_system(model_name, 0);
