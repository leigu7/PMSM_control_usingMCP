% Inspect Position Controller subsystem
clear; clc;
model_name = 'surface_mounted_pmsm_foc';
load_system(model_name);

fprintf('\n=== POSITION CONTROLLER SUBSYSTEM INSPECTION ===\n\n');
pc = find_system(model_name, 'SearchDepth', 1, 'Name', 'Position Controller');
if isempty(pc)
    fprintf('ERROR: Position Controller subsystem not found!\n');
    close_system(model_name, 0);
    return;
end
pc_path = pc{1};
fprintf('Path: %s\n\n', pc_path);

blocks = find_system(pc_path, 'SearchDepth', 1);
fprintf('Total blocks inside Position Controller: %d\n\n', length(blocks)-1);

fprintf('BLOCK LIST:\n');
for i = 2:length(blocks)
    [~,bname] = fileparts(blocks{i});
    btype = get_param(blocks{i}, 'BlockType');
    fprintf(' - %s (%s)\n', bname, btype);
end

% Check required ports
required_in = {'PositionRef','Theta_m'};
fprintf('\nINPUT PORTS CHECK:\n');
for i = 1:length(required_in)
    h = find_system(pc_path, 'SearchDepth',1, 'Name', required_in{i});
    if isempty(h)
        fprintf(' ⚠ Missing input port: %s\n', required_in{i});
    else
        fprintf(' ✓ Input port exists: %s\n', required_in{i});
    end
end

% Check controller blocks and output
fprintf('\nCONTROLLER BLOCKS CHECK:\n');
check_blocks = {'PosErr','Kp_pos','PosInt','Combine','Omega_ref'};
for i = 1:length(check_blocks)
    h = find_system(pc_path, 'SearchDepth',1, 'Name', check_blocks{i});
    if isempty(h)
        fprintf(' ⚠ Missing block: %s\n', check_blocks{i});
    else
        fprintf(' ✓ Found block: %s\n', check_blocks{i});
    end
end

% Output connection
fprintf('\nOUTPUT CONNECTIONS:\n');
outp = find_system(pc_path, 'SearchDepth',1,'BlockType','Outport');
for i = 1:length(outp)
    [~, oname] = fileparts(outp{i});
    try
        lineh = get_param(outp{i}, 'Line');
        if lineh == -1
            fprintf(' Outport %s -> UNCONNECTED\n', oname);
        else
            dsts = get_param(lineh, 'DstBlockHandle');
            dstnames = arrayfun(@(h)get_param(h,'Name'), dsts, 'UniformOutput',false);
            fprintf(' Outport %s -> %s\n', oname, strjoin(dstnames, ', '));
        end
    catch
        fprintf(' Outport %s -> unable to trace\n', oname);
    end
end

% Unconnected ports inside
fprintf('\nUNCONNECTED PORTS INSIDE POSITION CONTROLLER:\n');
ports = find_system(pc_path, 'FindAll', 'on', 'Type', 'port');
unconnected = 0;
for i = 1:length(ports)
    try
        line = get_param(ports(i), 'Line');
        if line == -1
            unconnected = unconnected + 1;
            blk = get_param(ports(i), 'Parent');
            [~,bn] = fileparts(blk);
            fprintf('  ⚠ Unconnected port in block: %s\n', bn);
        end
    catch
    end
end
if unconnected==0
    fprintf(' ✓ No unconnected ports found inside Position Controller\n');
else
    fprintf(' ⚠ Total unconnected ports: %d\n', unconnected);
end

fprintf('\n=== END POSITION CONTROLLER INSPECTION ===\n');
close_system(model_name,0);
