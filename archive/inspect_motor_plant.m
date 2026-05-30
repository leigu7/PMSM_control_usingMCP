% Detailed Motor Plant inspection
clear; clc;
model_name = 'surface_mounted_pmsm_foc';
load_system(model_name);

fprintf('\n=== MOTOR PLANT SUBSYSTEM DETAILED ANALYSIS ===\n\n');

motor_plant = find_system(model_name, 'SearchDepth', 1, 'Name', 'Motor Plant');
if isempty(motor_plant)
    fprintf('ERROR: Motor Plant subsystem not found!\n');
    close_system(model_name, 0);
    return;
end

mp = motor_plant{1};
fprintf('Motor Plant path: %s\n\n', mp);

% Get all blocks
all_blocks = find_system(mp, 'SearchDepth', 1);
fprintf('Total blocks in Motor Plant: %d\n\n', length(all_blocks)-1);

% Categorize
fprintf('BLOCK INVENTORY:\n');
fprintf('─────────────────────────────────────\n');

block_types = {};
for i = 2:length(all_blocks)
    btype = get_param(all_blocks{i}, 'BlockType');
    [~, bname] = fileparts(all_blocks{i});
    
    % Store for summary
    if ~any(strcmp(block_types, btype))
        block_types{end+1} = btype;
    end
    
    fprintf('%s (%s)\n', bname, btype);
end

% Check inputs and outputs
fprintf('\n\nINPUT/OUTPUT ANALYSIS:\n');
fprintf('─────────────────────────────────────\n');

inputs = find_system(mp, 'SearchDepth', 1, 'BlockType', 'Inport');
outputs = find_system(mp, 'SearchDepth', 1, 'BlockType', 'Outport');

fprintf('INPUT PORTS (%d):\n', length(inputs));
for i = 1:length(inputs)
    [~, name] = fileparts(inputs{i});
    port_num = get_param(inputs{i}, 'Port');
    fprintf('  [%s] Port #%s\n', name, port_num);
end

fprintf('\nOUTPUT PORTS (%d):\n', length(outputs));
for i = 1:length(outputs)
    [~, name] = fileparts(outputs{i});
    port_num = get_param(outputs{i}, 'Port');
    fprintf('  [%s] Port #%s\n', name, port_num);
end

% Check integrators
fprintf('\n\nINTEGRATOR ANALYSIS:\n');
fprintf('─────────────────────────────────────\n');
integrators = find_system(mp, 'SearchDepth', 1, 'BlockType', 'Integrator');
fprintf('Integrators found: %d\n', length(integrators));
for i = 1:length(integrators)
    [~, name] = fileparts(integrators{i});
    fprintf('  - %s\n', name);
end

% Check Products and Gains
fprintf('\n\nOPERATION BLOCKS:\n');
fprintf('─────────────────────────────────────\n');
products = find_system(mp, 'SearchDepth', 1, 'BlockType', 'Product');
gains = find_system(mp, 'SearchDepth', 1, 'BlockType', 'Gain');
sums = find_system(mp, 'SearchDepth', 1, 'BlockType', 'Sum');

fprintf('Product blocks: %d\n', length(products));
fprintf('Gain blocks: %d\n', length(gains));
fprintf('Sum blocks: %d\n', length(sums));

% Check signal connections to outputs
fprintf('\n\nOUTPUT CONNECTION ANALYSIS:\n');
fprintf('─────────────────────────────────────\n');
for i = 1:length(outputs)
    [~, out_name] = fileparts(outputs{i});
    port_num = str2double(get_param(outputs{i}, 'Port'));
    
    % Try to find what feeds this output
    try
        in_port_h = get_param(outputs{i}, 'Inport');
        line_h = get_param(in_port_h, 'Line');
        
        if line_h ~= -1
            src_blk_h = get_param(line_h, 'SrcBlockHandle');
            src_port = get_param(line_h, 'SrcPortHandle');
            
            % Get source block name
            src_name = get_param(src_blk_h, 'Name');
            fprintf('Output_%d (%s) <- %s\n', port_num, out_name, src_name);
        else
            fprintf('Output_%d (%s) <- UNCONNECTED!\n', port_num, out_name);
        end
    catch
        fprintf('Output_%d (%s) <- [unable to trace]\n', port_num, out_name);
    end
end

fprintf('\n=== END OF MOTOR PLANT ANALYSIS ===\n\n');

close_system(model_name, 0);
