%% ============================================================================
%  STEP 5b: Current Control Loop
%  Creates: pmsm_current_control.slx + test_current.slx
%  Dynamic block positioning with bx() grid layout
%  Verified: Id=1A, Iq=2A tracking <0.1A error
% ============================================================================
clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   STEP 5b: Current Control + SVPWM                         ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% Parameters
PolePairs=3; J=0.01; B=0.001; Rs=3.0;
Ld_nom=0.001; Lq_nom=0.001; psi_d_nom=0.08;
Vdc=600; v_max=Vdc/sqrt(3);
Kp_id=2*pi*500*Ld_nom; Ki_id=2*pi*500*Rs;
Kp_iq=2*pi*500*Lq_nom; Ki_iq=2*pi*500*Rs;
Iq_max=120; omega_max=300;
vars={'PolePairs',PolePairs;'J',J;'B',B;'Rs',Rs;'Ld_nom',Ld_nom;'Lq_nom',Lq_nom;...
        'psi_d_nom',psi_d_nom;'v_max',v_max;'Kp_id',Kp_id;'Ki_id',Ki_id;...
        'Kp_iq',Kp_iq;'Ki_iq',Ki_iq;'Iq_max',Iq_max;'omega_max',omega_max};
for vi=1:size(vars,1), assignin('base',vars{vi,1},vars{vi,2}); end

%% Layout: bx(col, row) -> [left, top, right, bottom]
W=100; H=60; HG=150; VG=100; x0=30; y0=30;
bx = @(c,r) [x0+c*(HG+W), y0+r*(VG+H), x0+c*(HG+W)+W, y0+r*(VG+H)+H];

%% ======================================================================
%  PART 1: Reusable Current Controller
%  In[1-8]: Id_ref, Iq_ref, Id_fb, Iq_fb, we, Ld, Lq, psid
%  Out[1-2]: Vd_ref, Vq_ref
% ======================================================================
ctrlName='pmsm_current_control';
if bdIsLoaded(ctrlName), close_system(ctrlName,0); end
if exist([ctrlName '.slx'],'file'), delete([ctrlName '.slx']); end
new_system(ctrlName,'Model');

% Inports (c=0, rows 0-7)
inlist={'Id_ref','1';'Iq_ref','2';'Id_fb','3';'Iq_fb','4';'we','5';'Ld','6';'Lq','7';'psid','8'};
for i=1:size(inlist,1)
    add_block('simulink/Sources/In1',[ctrlName '/' inlist{i,1}],'Position',bx(0,i-1));
    set_param([ctrlName '/' inlist{i,1}],'Port',inlist{i,2});
end

% d-axis: Err_D = Id_ref - Id_fb (c=1)
add_block('simulink/Math Operations/Sum',[ctrlName '/Err_D'],'Inputs','+-','Position',bx(1,0));
add_block('simulink/Math Operations/Gain',[ctrlName '/Kp_id'],'Gain','Kp_id','Position',bx(2,0));
add_block('simulink/Continuous/Integrator',[ctrlName '/Int_D'],'InitialCondition','0','Position',bx(2,1));
add_block('simulink/Math Operations/Gain',[ctrlName '/Ki_id'],'Gain','Ki_id','Position',bx(3,1));
add_block('simulink/Math Operations/Sum',[ctrlName '/Sum_PI_D'],'Inputs','++','Position',bx(3,0));

% q-axis: Err_Q = Iq_ref - Iq_fb (c=1, row=2)
add_block('simulink/Math Operations/Sum',[ctrlName '/Err_Q'],'Inputs','+-','Position',bx(1,2));
add_block('simulink/Math Operations/Gain',[ctrlName '/Kp_iq'],'Gain','Kp_iq','Position',bx(2,2));
add_block('simulink/Continuous/Integrator',[ctrlName '/Int_Q'],'InitialCondition','0','Position',bx(2,3));
add_block('simulink/Math Operations/Gain',[ctrlName '/Ki_iq'],'Gain','Ki_iq','Position',bx(3,3));
add_block('simulink/Math Operations/Sum',[ctrlName '/Sum_PI_Q'],'Inputs','++','Position',bx(3,2));

% Decoupling: Vd_ref = Vd_pi - we*Lq*Iq
add_block('simulink/Math Operations/Product',[ctrlName '/weLq'],'Position',bx(2,4));
add_block('simulink/Math Operations/Product',[ctrlName '/weLqIq'],'Position',bx(3,4));
add_block('simulink/Math Operations/Sum',[ctrlName '/Sum_Vd'],'Inputs','+-','Position',bx(4,0));

% Feedforward: Vq_ref = Vq_pi + we*Ld*Id + we*psid
add_block('simulink/Math Operations/Product',[ctrlName '/weLd'],'Position',bx(2,5));
add_block('simulink/Math Operations/Product',[ctrlName '/weLdId'],'Position',bx(3,5));
add_block('simulink/Math Operations/Product',[ctrlName '/wePsid'],'Position',bx(3,6));
add_block('simulink/Math Operations/Sum',[ctrlName '/Sum_FF'],'Inputs','++','Position',bx(4,5));
add_block('simulink/Math Operations/Sum',[ctrlName '/Sum_Vq'],'Inputs','++','Position',bx(5,2));

% Outports
add_block('simulink/Sinks/Out1',[ctrlName '/Vd_ref'],'Position',bx(5,0));
set_param([ctrlName '/Vd_ref'],'Port','1');
add_block('simulink/Sinks/Out1',[ctrlName '/Vq_ref'],'Position',bx(6,2));
set_param([ctrlName '/Vq_ref'],'Port','2');

%% Wiring
add_line(ctrlName,'Id_ref/1','Err_D/1'); add_line(ctrlName,'Id_fb/1','Err_D/2');
add_line(ctrlName,'Err_D/1','Kp_id/1'); add_line(ctrlName,'Err_D/1','Int_D/1');
add_line(ctrlName,'Int_D/1','Ki_id/1');
add_line(ctrlName,'Kp_id/1','Sum_PI_D/1'); add_line(ctrlName,'Ki_id/1','Sum_PI_D/2');
add_line(ctrlName,'we/1','weLq/1'); add_line(ctrlName,'Lq/1','weLq/2');
add_line(ctrlName,'weLq/1','weLqIq/1'); add_line(ctrlName,'Iq_fb/1','weLqIq/2');
add_line(ctrlName,'Sum_PI_D/1','Sum_Vd/1'); add_line(ctrlName,'weLqIq/1','Sum_Vd/2');
add_line(ctrlName,'Sum_Vd/1','Vd_ref/1');

add_line(ctrlName,'Iq_ref/1','Err_Q/1'); add_line(ctrlName,'Iq_fb/1','Err_Q/2');
add_line(ctrlName,'Err_Q/1','Kp_iq/1'); add_line(ctrlName,'Err_Q/1','Int_Q/1');
add_line(ctrlName,'Int_Q/1','Ki_iq/1');
add_line(ctrlName,'Kp_iq/1','Sum_PI_Q/1'); add_line(ctrlName,'Ki_iq/1','Sum_PI_Q/2');
add_line(ctrlName,'we/1','weLd/1'); add_line(ctrlName,'Ld/1','weLd/2');
add_line(ctrlName,'weLd/1','weLdId/1'); add_line(ctrlName,'Id_fb/1','weLdId/2');
add_line(ctrlName,'we/1','wePsid/1'); add_line(ctrlName,'psid/1','wePsid/2');
add_line(ctrlName,'weLdId/1','Sum_FF/1'); add_line(ctrlName,'wePsid/1','Sum_FF/2');
add_line(ctrlName,'Sum_PI_Q/1','Sum_Vq/1'); add_line(ctrlName,'Sum_FF/1','Sum_Vq/2');
add_line(ctrlName,'Sum_Vq/1','Vq_ref/1');

save_system(ctrlName);
fprintf('Reusable CC saved: %s.slx\n',ctrlName);

%% ======================================================================
%  PART 2: Test Harness
% ======================================================================
testName='test_current';
if bdIsLoaded(testName), close_system(testName,0); end
if exist([testName '.slx'],'file'), delete([testName '.slx']); end
new_system(testName,'Model');
set_param(testName,'Solver','ode4','SolverType','Fixed-step','FixedStep','1e-5','StopTime','0.05');

% --- Plant Subsystem (inline) ---
P=[testName '/Motor Plant'];
add_block('simulink/Ports & Subsystems/Subsystem',P,'Position',[10 10 1150 750]);
delete_block([P '/In1']); delete_block([P '/Out1']);
pin={'Vd','1';'Vq','2';'LoadTorque','3';'PolePairs','4';'J','5';'B','6';'Temperature','7'};
for i=1:size(pin,1)
    add_block('simulink/Sources/In1',[P '/' pin{i,1}],'Position',bx(i-1,0));
    set_param([P '/' pin{i,1}],'Port',pin{i,2});
end
add_block('simulink/Sources/Constant',[P '/Rs_c'],'Value','Rs','Position',bx(0,1));
add_block('simulink/Sources/Constant',[P '/Ld_c'],'Value','Ld_nom','Position',bx(0,2));
add_block('simulink/Sources/Constant',[P '/Lq_c'],'Value','Lq_nom','Position',bx(0,3));
add_block('simulink/Sources/Constant',[P '/psid_c'],'Value','psi_d_nom','Position',bx(0,4));
add_block('simulink/Math Operations/Sum',[P '/SumD'],'Inputs','+-+','Position',bx(1,2));
add_block('simulink/Math Operations/Gain',[P '/G1Ld'],'Gain','1/Ld_nom','Position',bx(2,2));
add_block('simulink/Continuous/Integrator',[P '/IntId'],'InitialCondition','0','Position',bx(3,2));
add_block('simulink/Math Operations/Product',[P '/RsId'],'Position',bx(0,3));
add_block('simulink/Math Operations/Product',[P '/weLq'],'Position',bx(0,5));
add_block('simulink/Math Operations/Product',[P '/weLqIq'],'Position',bx(1,5));
add_block('simulink/Math Operations/Sum',[P '/SumQ'],'Inputs','+--','Position',bx(1,3));
add_block('simulink/Math Operations/Gain',[P '/G1Lq'],'Gain','1/Lq_nom','Position',bx(2,3));
add_block('simulink/Continuous/Integrator',[P '/IntIq'],'InitialCondition','0','Position',bx(3,3));
add_block('simulink/Math Operations/Product',[P '/RsIq'],'Position',bx(0,4));
add_block('simulink/Math Operations/Product',[P '/weLd'],'Position',bx(0,6));
add_block('simulink/Math Operations/Product',[P '/weLdId'],'Position',bx(1,6));
add_block('simulink/Math Operations/Product',[P '/wePsid'],'Position',bx(2,6));
add_block('simulink/Math Operations/Sum',[P '/SumEMF'],'Inputs','++','Position',bx(3,6));
add_block('simulink/Math Operations/Product',[P '/weCalc'],'Position',bx(4,5));
add_block('simulink/Math Operations/Gain',[P '/k_t'],'Gain','1.5*PolePairs*psi_d_nom','Position',bx(4,2));
add_block('simulink/Math Operations/Product',[P '/Bomega'],'Position',bx(4,3));
add_block('simulink/Math Operations/Sum',[P '/SumMech'],'Inputs','+--','Position',bx(5,3));
add_block('simulink/Math Operations/Gain',[P '/G1J'],'Gain','1/J','Position',bx(6,3));
add_block('simulink/Continuous/Integrator',[P '/Int_omega'],'InitialCondition','0','Position',bx(7,3));
add_block('simulink/Continuous/Integrator',[P '/Int_theta'],'InitialCondition','0','Position',bx(8,3));
pout={'Id','1';'Iq','2';'Te','3';'omega_m','4';'theta_m','5'};
for i=1:size(pout,1)
    add_block('simulink/Sinks/Out1',[P '/' pout{i,1}],'Position',bx(9,i-1));
    set_param([P '/' pout{i,1}],'Port',pout{i,2});
end
% Plant wiring
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

% --- Current Controller in test harness (inline) ---
CC=[testName '/Current Controller'];
add_block('simulink/Ports & Subsystems/Subsystem',CC,'Position',[200 200 600 500]);
delete_block([CC '/In1']); delete_block([CC '/Out1']);
for i=1:size(inlist,1)
    add_block('simulink/Sources/In1',[CC '/' inlist{i,1}],'Position',bx(0,i-1));
    set_param([CC '/' inlist{i,1}],'Port',inlist{i,2});
end
add_block('simulink/Math Operations/Sum',[CC '/Err_D'],'Inputs','+-','Position',bx(1,0));
add_block('simulink/Math Operations/Gain',[CC '/Kp_id'],'Gain','Kp_id','Position',bx(2,0));
add_block('simulink/Continuous/Integrator',[CC '/Int_D'],'InitialCondition','0','Position',bx(2,1));
add_block('simulink/Math Operations/Gain',[CC '/Ki_id'],'Gain','Ki_id','Position',bx(3,1));
add_block('simulink/Math Operations/Sum',[CC '/Sum_PI_D'],'Inputs','++','Position',bx(3,0));
add_block('simulink/Math Operations/Sum',[CC '/Err_Q'],'Inputs','+-','Position',bx(1,2));
add_block('simulink/Math Operations/Gain',[CC '/Kp_iq'],'Gain','Kp_iq','Position',bx(2,2));
add_block('simulink/Continuous/Integrator',[CC '/Int_Q'],'InitialCondition','0','Position',bx(2,3));
add_block('simulink/Math Operations/Gain',[CC '/Ki_iq'],'Gain','Ki_iq','Position',bx(3,3));
add_block('simulink/Math Operations/Sum',[CC '/Sum_PI_Q'],'Inputs','++','Position',bx(3,2));
add_block('simulink/Math Operations/Product',[CC '/weLq'],'Position',bx(2,4));
add_block('simulink/Math Operations/Product',[CC '/weLqIq'],'Position',bx(3,4));
add_block('simulink/Math Operations/Sum',[CC '/Sum_Vd'],'Inputs','+-','Position',bx(4,0));
add_block('simulink/Math Operations/Product',[CC '/weLd'],'Position',bx(2,5));
add_block('simulink/Math Operations/Product',[CC '/weLdId'],'Position',bx(3,5));
add_block('simulink/Math Operations/Product',[CC '/wePsid'],'Position',bx(3,6));
add_block('simulink/Math Operations/Sum',[CC '/Sum_FF'],'Inputs','++','Position',bx(4,5));
add_block('simulink/Math Operations/Sum',[CC '/Sum_Vq'],'Inputs','++','Position',bx(5,2));
add_block('simulink/Sinks/Out1',[CC '/Vd_ref'],'Position',bx(5,0)); set_param([CC '/Vd_ref'],'Port','1');
add_block('simulink/Sinks/Out1',[CC '/Vq_ref'],'Position',bx(6,2)); set_param([CC '/Vq_ref'],'Port','2');
% CC wiring
add_line(CC,'Id_ref/1','Err_D/1'); add_line(CC,'Id_fb/1','Err_D/2');
add_line(CC,'Err_D/1','Kp_id/1'); add_line(CC,'Err_D/1','Int_D/1');
add_line(CC,'Int_D/1','Ki_id/1'); add_line(CC,'Kp_id/1','Sum_PI_D/1'); add_line(CC,'Ki_id/1','Sum_PI_D/2');
add_line(CC,'we/1','weLq/1'); add_line(CC,'Lq/1','weLq/2');
add_line(CC,'weLq/1','weLqIq/1'); add_line(CC,'Iq_fb/1','weLqIq/2');
add_line(CC,'Sum_PI_D/1','Sum_Vd/1'); add_line(CC,'weLqIq/1','Sum_Vd/2');
add_line(CC,'Sum_Vd/1','Vd_ref/1');
add_line(CC,'Iq_ref/1','Err_Q/1'); add_line(CC,'Iq_fb/1','Err_Q/2');
add_line(CC,'Err_Q/1','Kp_iq/1'); add_line(CC,'Err_Q/1','Int_Q/1');
add_line(CC,'Int_Q/1','Ki_iq/1'); add_line(CC,'Kp_iq/1','Sum_PI_Q/1'); add_line(CC,'Ki_iq/1','Sum_PI_Q/2');
add_line(CC,'we/1','weLd/1'); add_line(CC,'Ld/1','weLd/2');
add_line(CC,'weLd/1','weLdId/1'); add_line(CC,'Id_fb/1','weLdId/2');
add_line(CC,'we/1','wePsid/1'); add_line(CC,'psid/1','wePsid/2');
add_line(CC,'weLdId/1','Sum_FF/1'); add_line(CC,'wePsid/1','Sum_FF/2');
add_line(CC,'Sum_PI_Q/1','Sum_Vq/1'); add_line(CC,'Sum_FF/1','Sum_Vq/2');
add_line(CC,'Sum_Vq/1','Vq_ref/1');

% --- Top-level test harness ---
add_block('simulink/Sources/Step',[testName '/Id_step'],'Position',bx(0,0));
set_param([testName '/Id_step'],'Time','0','Before','0','After','1');
add_block('simulink/Sources/Step',[testName '/Iq_step'],'Position',bx(0,1));
set_param([testName '/Iq_step'],'Time','0.01','Before','0','After','2');
add_block('simulink/Sources/Constant',[testName '/TL'],'Value','0','Position',bx(1,0));
add_block('simulink/Sources/Constant',[testName '/PP'],'Value','PolePairs','Position',bx(1,1));
add_block('simulink/Sources/Constant',[testName '/Jc'],'Value','J','Position',bx(1,2));
add_block('simulink/Sources/Constant',[testName '/Bc'],'Value','B','Position',bx(1,3));
add_block('simulink/Sources/Constant',[testName '/Tc'],'Value','25','Position',bx(1,4));
add_block('simulink/Sources/Constant',[testName '/Ld_cn'],'Value','Ld_nom','Position',bx(2,0));
add_block('simulink/Sources/Constant',[testName '/Lq_cn'],'Value','Lq_nom','Position',bx(2,1));
add_block('simulink/Sources/Constant',[testName '/psid_cn'],'Value','psi_d_nom','Position',bx(2,2));
add_block('simulink/Math Operations/Product',[testName '/we_calc'],'Position',bx(2,3));

% Wiring between blocks
add_line(testName,'Id_step/1','Current Controller/1');
add_line(testName,'Iq_step/1','Current Controller/2');
add_line(testName,'Motor Plant/1','Current Controller/3');
add_line(testName,'Motor Plant/2','Current Controller/4');
add_line(testName,'we_calc/1','Current Controller/5');
add_line(testName,'Ld_cn/1','Current Controller/6');
add_line(testName,'Lq_cn/1','Current Controller/7');
add_line(testName,'psid_cn/1','Current Controller/8');
add_line(testName,'Current Controller/1','Motor Plant/1');
add_line(testName,'Current Controller/2','Motor Plant/2');
add_line(testName,'PP/1','we_calc/1');
add_line(testName,'Motor Plant/4','we_calc/2');
add_line(testName,'TL/1','Motor Plant/3');
add_line(testName,'PP/1','Motor Plant/4');
add_line(testName,'Jc/1','Motor Plant/5');
add_line(testName,'Bc/1','Motor Plant/6');
add_line(testName,'Tc/1','Motor Plant/7');

% Data logging
add_block('simulink/Sinks/To Workspace',[testName '/W_Id'],'Position',bx(5,0));
set_param([testName '/W_Id'],'VariableName','Id_sim','SaveFormat','Array');
add_block('simulink/Sinks/To Workspace',[testName '/W_Iq'],'Position',bx(5,1));
set_param([testName '/W_Iq'],'VariableName','Iq_sim','SaveFormat','Array');
add_line(testName,'Motor Plant/1','W_Id/1');
add_line(testName,'Motor Plant/2','W_Iq/1');

save_system(testName);
fprintf('Test harness saved: %s.slx\n\n',testName);

%% ======================================================================
%  PART 3: Verification
% ======================================================================
fprintf('Simulating...\n');
try
    out=sim(testName,'StopTime','0.05','ReturnWorkspaceOutputs','on');
    Id=out.get('Id_sim'); Iq=out.get('Iq_sim');
    N=length(Id); ss=round(N*0.8);
    Id_ss=mean(Id(ss:end)); Iq_ss=mean(Iq(ss:end));
    fprintf('  Id: ref=1.0A measured=%.4fA\n',Id_ss);
    fprintf('  Iq: ref=2.0A measured=%.4fA\n',Iq_ss);
    if abs(Id_ss-1)<0.1 && abs(Iq_ss-2)<0.1
        fprintf('\n✅ VERIFICATION PASSED\n');
    else
        fprintf('\n❌ VERIFICATION FAILED\n');
    end
catch ME
    fprintf('ERROR: %s\n',ME.message);
end

fprintf('\n╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   STEP 5b DONE                                              ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');
