% Inspect Current Controller subsystem
clear; clc;
model_name = 'surface_mounted_pmsm_foc';
load_system(model_name);

fprintf('\n=== CURRENT CONTROLLER SUBSYSTEM INSPECTION ===\n\n');
cc = find_system(model_name, 'SearchDepth', 1, 'Name', 'Current Controller');
if isempty(cc)
    fprintf('ERROR: Current Controller subsystem not found!\n');
    close_system(model_name, 0);
    return;
end

cc_path = cc{1};
fprintf('Path: %s\n\n', cc_path);

blocks = find_system(cc_path, 'SearchDepth', 1);
fprintf('Total blocks inside Current Controller: %d\n\n', length(blocks)-1);

fprintf('BLOCK LIST:\n');
for i = 2:length(blocks)
    [~,bname] = fileparts(blocks{i});
    btype = get_param(blocks{i}, 'BlockType');
    fprintf(' - %s (%s)\n', bname, btype);
end

% Check for required inports
required_in = {'IdRef','IqRef','Id','Iq','omega_e','Ld','Lq','psi_d'};
fprintf('\nINPUT PORTS CHECK:\n');
for i = 1:length(required_in)
    h = find_system(cc_path, 'SearchDepth',1, 'Name', required_in{i});
    if isempty(h)
        fprintf(' ⚠ Missing input port: %s\n', required_in{i});
    else
        fprintf(' ✓ Input port exists: %s\n', required_in{i});
    end
end

% Check for PI blocks and outputs
fprintf('\nCONTROLLER BLOCKS CHECK:\n');
check_blocks = {'Kp_id','IdInt','Kp_iq','IqInt','Vd_sum','Vq_sum','Vd','Vq'};
for i = 1:length(check_blocks)
    h = find_system(cc_path, 'SearchDepth',1, 'Name', check_blocks{i});
    if isempty(h)
        fprintf(' ⚠ Missing block: %s\n', check_blocks{i});
    else
        fprintf(' ✓ Found block: %s\n', check_blocks{i});
    end
end

% Check connectivity: are Vd and Vq outputs connected to top level?
fprintf('\nOUTPUT CONNECTIONS:\n');
try
    % get top model lines
    td = find_system(model_name, 'FindAll','on','Type','line');
    % find lines where src block is inside current controller and dst is outside
    % iterate over outports of Current Controller
    outp = find_system(cc_path, 'SearchDepth',1,'BlockType','Outport');
    for i = 1:length(outp)
        portnum = get_param(outp{i}, 'Port');
        [~, oname] = fileparts(outp{i});
        fprintf(' Outport %s (port %s) -> ', oname, portnum);
        % find corresponding top-level connection
        try
            % Resolve full block path and find its parent mapping
            full = get_param(outp{i}, 'Handle');
            lineh = get_param(outp{i}, 'Line');
            if lineh == -1
                fprintf(' UNCONNECTED\n');
            else
                dsts = get_param(lineh, 'DstBlockHandle');
                dstnames = arrayfun(@(h)get_param(h,'Name'), dsts, 'UniformOutput',false);
                fprintf('%s\n', strjoin(dstnames, ', '));
            end
        catch
            fprintf(' unable to trace\n');
        end
    end
catch
    fprintf('Could not analyze output connections reliably.\n');
end

% Check for unconnected ports inside
fprintf('\nSEARCH FOR UNCONNECTED PORTS INSIDE CURRENT CONTROLLER:\n');
ports = find_system(cc_path, 'FindAll', 'on', 'Type', 'port');
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
    fprintf(' ✓ No unconnected ports found inside Current Controller\n');
else
    fprintf(' ⚠ Total unconnected ports: %d\n', unconnected);
end

fprintf('\n=== END CURRENT CONTROLLER INSPECTION ===\n');
close_system(model_name,0);
