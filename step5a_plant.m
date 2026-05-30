%% ============================================================================
%  STEP 5a: Motor Plant Model
%  Creates: pmsm_plant.slx
%  Dynamic block positioning with bx() grid layout
%  CRITICAL: All port numbers are 1-based.
% ============================================================================
clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   STEP 5a: PMSM Plant Model                                ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% Parameters
PolePairs=3; J=0.01; B=0.001; Rs=3.0;
Ld_nom=0.001; Lq_nom=0.001; psi_d_nom=0.08;
Vdc=600; v_max=Vdc/sqrt(3);
assignin('base','PolePairs',PolePairs); assignin('base','J',J);
assignin('base','B',B); assignin('base','Rs',Rs);
assignin('base','Ld_nom',Ld_nom); assignin('base','Lq_nom',Lq_nom);
assignin('base','psi_d_nom',psi_d_nom); assignin('base','v_max',v_max);
fprintf('PolePairs=%d, J=%.4f, B=%.4f, Rs=%.1f, Ld=%.4f, Lq=%.4f\n\n',...
        PolePairs, J, B, Rs, Ld_nom, Lq_nom);

%% Layout: bx(col, row) -> [left, top, right, bottom]
W=100; H=60; HG=150; VG=100; x0=30; y0=30;
bx = @(c,r) [x0+c*(HG+W), y0+r*(VG+H), x0+c*(HG+W)+W, y0+r*(VG+H)+H];

%% Build
modelName='pmsm_plant';
if bdIsLoaded(modelName), close_system(modelName,0); end
if exist([modelName '.slx'],'file'), delete([modelName '.slx']); end
new_system(modelName,'Model');
P=[modelName '/Motor Plant'];
add_block('simulink/Ports & Subsystems/Subsystem',P,'Position',[10 10 1150 750]);
delete_block([P '/In1']); delete_block([P '/Out1']);

% Inports (row 0)
pin={'Vd','1';'Vq','2';'LoadTorque','3';'PolePairs','4';'J','5';'B','6';'Temperature','7'};
for i=1:size(pin,1)
    add_block('simulink/Sources/In1',[P '/' pin{i,1}],'Position',bx(i-1,0));
    set_param([P '/' pin{i,1}],'Port',pin{i,2});
end

% Constants (row 1, shifted right)
add_block('simulink/Sources/Constant',[P '/Rs_c'],'Value','Rs','Position',bx(0,1));
add_block('simulink/Sources/Constant',[P '/Ld_c'],'Value','Ld_nom','Position',bx(0,2));
add_block('simulink/Sources/Constant',[P '/Lq_c'],'Value','Lq_nom','Position',bx(0,3));
add_block('simulink/Sources/Constant',[P '/psid_c'],'Value','psi_d_nom','Position',bx(0,4));

% d-axis: Vd + weLqIq - RsId -> 1/Ld -> Int -> Id
add_block('simulink/Math Operations/Sum',[P '/SumD'],'Inputs','+-+','Position',bx(1,2));
add_block('simulink/Math Operations/Gain',[P '/G1Ld'],'Gain','1/Ld_nom','Position',bx(2,2));
add_block('simulink/Continuous/Integrator',[P '/IntId'],'InitialCondition','0','Position',bx(3,2));
add_block('simulink/Math Operations/Product',[P '/RsId'],'Position',bx(0,3));
add_block('simulink/Math Operations/Product',[P '/weLq'],'Position',bx(0,5));
add_block('simulink/Math Operations/Product',[P '/weLqIq'],'Position',bx(1,5));

% q-axis: Vq - RsIq - (weLdId+wePsid) -> 1/Lq -> Int -> Iq
add_block('simulink/Math Operations/Sum',[P '/SumQ'],'Inputs','+--','Position',bx(1,3));
add_block('simulink/Math Operations/Gain',[P '/G1Lq'],'Gain','1/Lq_nom','Position',bx(2,3));
add_block('simulink/Continuous/Integrator',[P '/IntIq'],'InitialCondition','0','Position',bx(3,3));
add_block('simulink/Math Operations/Product',[P '/RsIq'],'Position',bx(0,4));
add_block('simulink/Math Operations/Product',[P '/weLd'],'Position',bx(0,6));
add_block('simulink/Math Operations/Product',[P '/weLdId'],'Position',bx(1,6));
add_block('simulink/Math Operations/Product',[P '/wePsid'],'Position',bx(2,6));
add_block('simulink/Math Operations/Sum',[P '/SumEMF'],'Inputs','++','Position',bx(3,6));

% Electrical speed
add_block('simulink/Math Operations/Product',[P '/weCalc'],'Position',bx(4,5));

% Torque & Mech
add_block('simulink/Math Operations/Gain',[P '/k_t'],'Gain','1.5*PolePairs*psi_d_nom','Position',bx(4,2));
add_block('simulink/Math Operations/Product',[P '/Bomega'],'Position',bx(4,3));
add_block('simulink/Math Operations/Sum',[P '/SumMech'],'Inputs','+--','Position',bx(5,3));
add_block('simulink/Math Operations/Gain',[P '/G1J'],'Gain','1/J','Position',bx(6,3));
add_block('simulink/Continuous/Integrator',[P '/Int_omega'],'InitialCondition','0','Position',bx(7,3));
add_block('simulink/Continuous/Integrator',[P '/Int_theta'],'InitialCondition','0','Position',bx(8,3));

% Outports (row 0, far right)
pout={'Id','1';'Iq','2';'Te','3';'omega_m','4';'theta_m','5'};
for i=1:size(pout,1)
    add_block('simulink/Sinks/Out1',[P '/' pout{i,1}],'Position',bx(9,i-1));
    set_param([P '/' pout{i,1}],'Port',pout{i,2});
end

%% Wiring
add_line(P,'Vd/1','SumD/1');
add_line(P,'IntId/1','RsId/1'); add_line(P,'Rs_c/1','RsId/2'); add_line(P,'RsId/1','SumD/2');
add_line(P,'weLqIq/1','SumD/3');
add_line(P,'SumD/1','G1Ld/1'); add_line(P,'G1Ld/1','IntId/1'); add_line(P,'IntId/1','Id/1');

add_line(P,'Vq/1','SumQ/1');
add_line(P,'IntIq/1','RsIq/1'); add_line(P,'Rs_c/1','RsIq/2'); add_line(P,'RsIq/1','SumQ/2');
add_line(P,'SumEMF/1','SumQ/3');
add_line(P,'SumQ/1','G1Lq/1'); add_line(P,'G1Lq/1','IntIq/1'); add_line(P,'IntIq/1','Iq/1');

add_line(P,'PolePairs/1','weCalc/1'); add_line(P,'Int_omega/1','weCalc/2');
add_line(P,'weCalc/1','weLq/1'); add_line(P,'Lq_c/1','weLq/2');
add_line(P,'weLq/1','weLqIq/1'); add_line(P,'IntIq/1','weLqIq/2');
add_line(P,'weCalc/1','weLd/1'); add_line(P,'Ld_c/1','weLd/2');
add_line(P,'weLd/1','weLdId/1'); add_line(P,'IntId/1','weLdId/2');
add_line(P,'weCalc/1','wePsid/1'); add_line(P,'psid_c/1','wePsid/2');
add_line(P,'weLdId/1','SumEMF/1'); add_line(P,'wePsid/1','SumEMF/2');

add_line(P,'IntIq/1','k_t/1');
add_line(P,'k_t/1','SumMech/1'); add_line(P,'k_t/1','Te/1');
add_line(P,'Int_omega/1','Bomega/1'); add_line(P,'B/1','Bomega/2');
add_line(P,'Bomega/1','SumMech/2'); add_line(P,'LoadTorque/1','SumMech/3');
add_line(P,'SumMech/1','G1J/1'); add_line(P,'G1J/1','Int_omega/1');
add_line(P,'Int_omega/1','Int_theta/1');
add_line(P,'Int_omega/1','omega_m/1'); add_line(P,'Int_theta/1','theta_m/1');

save_system(modelName);
fprintf('Saved: %s.slx\n\n',modelName);

%% Verify
inp=find_system(P,'SearchDepth',1,'BlockType','Inport');
fprintf('Inports:\n'); for i=1:length(inp), fprintf('  Port %s: %s\n',get_param(inp{i},'Port'),get_param(inp{i},'Name')); end
outp=find_system(P,'SearchDepth',1,'BlockType','Outport');
fprintf('Outports:\n'); for i=1:length(outp), fprintf('  Port %s: %s\n',get_param(outp{i},'Port'),get_param(outp{i},'Name')); end

fprintf('\n╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   STEP 5a DONE                                              ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');
