model = 'tmp_debug_mdl';
if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model, 'Model');
add_block('simulink/Ports & Subsystems/Subsystem', [model '/sub'], 'Position', [100 100 300 250]);
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/sub/fcn'], 'Position', [120 120 240 200]);
try
    blockType = get_param([model '/sub/fcn'], 'BlockType');
    disp(['BlockType=' blockType]);
    fparam = get_param([model '/sub/fcn'], 'FunctionName');
    disp(['FunctionName=' fparam]);
catch ME
    disp(ME.message);
end
close_system(model, 0);
