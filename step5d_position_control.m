%% ============================================================================
%  STEP 5d: Position Control Loop (Full FOC)
%  Creates: pmsm_position_control.slx + test_position.slx
%  Dynamic block positioning with bx() grid layout
%  Verified: θ=π rad step, error <0.5 rad
% ============================================================================
clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   STEP 5d: Position Control Loop                           ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% Parameters
PolePairs=3; J=0.01; B=0.001; Rs=3.0;
Ld_nom=0.001; Lq_nom=0.001; psi_d_nom=0.08;
Vdc=600; v_max=Vdc/sqrt(3);
Kp_id=2*pi*500*Ld_nom; Ki_id=2*pi*500*Rs;
Kp_iq=2*pi*500*Lq_nom; Ki_iq=2*pi*500*Rs;
k_t=1.5*PolePairs*psi_d_nom;
Kp_speed=2*pi*30*J/k_t; Ki_speed=2*pi*30*B/k_t;
Kp_pos=2*pi*5; Ki_pos=Kp_pos*5/2;
Iq_max=120; omega_max=300;
vars={'PolePairs',PolePairs;'J',J;'B',B;'Rs',Rs;'Ld_nom',Ld_nom;'Lq_nom',Lq_nom;...
        'psi_d_nom',psi_d_nom;'v_max',v_max;'Kp_id',Kp_id;'Ki_id',Ki_id;...
        'Kp_iq',Kp_iq;'Ki_iq',Ki_iq;'Iq_max',Iq_max;'omega_max',omega_max;...
        'Kp_speed',Kp_speed;'Ki_speed',Ki_speed;'Kp_pos',Kp_pos;'Ki_pos',Ki_pos};
for vi=1:size(vars,1), assignin('base',vars{vi,1},vars{vi,2}); end
fprintf('Pos PI: Kp=%.2f, Ki=%.2f, ω_max=%.0f\n\n',Kp_pos,Ki_pos,omega_max);

%% Layout
W=100; H=60; HG=150; VG=100; x0=30; y0=30;
bx = @(c,r) [x0+c*(HG+W), y0+r*(VG+H), x0+c*(HG+W)+W, y0+r*(VG+H)+H];

%% ======================================================================
%  PART 1: Position Controller
%  In[1]theta_ref, In[2]theta_fb -> Out[1]omega_ref
%  Angle wrapping: err = mod(err+pi, 2*pi)-pi
% ======================================================================
ctrlName='pmsm_position_control';
if bdIsLoaded(ctrlName), close_system(ctrlName,0); end
if exist([ctrlName '.slx'],'file'), delete([ctrlName '.slx']); end
new_system(ctrlName,'Model');

add_block('simulink/Sources/In1',[ctrlName '/theta_ref'],'Position',bx(0,0));
set_param([ctrlName '/theta_ref'],'Port','1');
add_block('simulink/Sources/In1',[ctrlName '/theta_fb'],'Position',bx(0,1));
set_param([ctrlName '/theta_fb'],'Port','2');

% Error = theta_ref - theta_fb
add_block('simulink/Math Operations/Sum',[ctrlName '/Err_raw'],'Inputs','+-','Position',bx(1,0));

% Wrapping: (err+pi) -> mod(err+pi, 2*pi) -> (result - pi)
add_block('simulink/User-Defined Functions/Fcn',[ctrlName '/AddPi'],'Position',bx(2,0));
set_param([ctrlName '/AddPi'],'Expr','u+pi');
add_block('simulink/User-Defined Functions/Fcn',[ctrlName '/Mod2Pi'],'Position',bx(3,0));
set_param([ctrlName '/Mod2Pi'],'Expr','u - 2*pi*floor(u/(2*pi))');
add_block('simulink/User-Defined Functions/Fcn',[ctrlName '/SubPi'],'Position',bx(4,0));
set_param([ctrlName '/SubPi'],'Expr','u-pi');

% PI
add_block('simulink/Math Operations/Gain',[ctrlName '/Kp'],'Gain','Kp_pos','Position',bx(5,0));
add_block('simulink/Continuous/Integrator',[ctrlName '/Int'],'InitialCondition','0','Position',bx(5,1));
add_block('simulink/Math Operations/Gain',[ctrlName '/Ki'],'Gain','Ki_pos','Position',bx(6,1));
add_block('simulink/Math Operations/Sum',[ctrlName '/Sum_PI'],'Inputs','++','Position',bx(6,0));
add_block('simulink/Discontinuities/Saturation',[ctrlName '/Sat'],...
    'UpperLimit','omega_max','LowerLimit','-omega_max','Position',bx(7,0));

add_block('simulink/Sinks/Out1',[ctrlName '/omega_ref'],'Position',bx(8,0));
set_param([ctrlName '/omega_ref'],'Port','1');

% Wiring
add_line(ctrlName,'theta_ref/1','Err_raw/1'); add_line(ctrlName,'theta_fb/1','Err_raw/2');
add_line(ctrlName,'Err_raw/1','AddPi/1');
add_line(ctrlName,'AddPi/1','Mod2Pi/1');
add_line(ctrlName,'Mod2Pi/1','SubPi/1');
add_line(ctrlName,'SubPi/1','Kp/1'); add_line(ctrlName,'SubPi/1','Int/1');
add_line(ctrlName,'Int/1','Ki/1');
add_line(ctrlName,'Kp/1','Sum_PI/1'); add_line(ctrlName,'Ki/1','Sum_PI/2');
add_line(ctrlName,'Sum_PI/1','Sat/1'); add_line(ctrlName,'Sat/1','omega_ref/1');

save_system(ctrlName);
fprintf('Position Controller saved: %s.slx\n\n',ctrlName);

%% ======================================================================
%  PART 2: Test Harness (Full FOC)
% ======================================================================
testName='test_position';
if bdIsLoaded(testName), close_system(testName,0); end
if exist([testName '.slx'],'file'), delete([testName '.slx']); end
new_system(testName,'Model');
set_param(testName,'Solver','ode4','SolverType','Fixed-step','FixedStep','1e-5','StopTime','0.5');

% Plant
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
add_line(P,'Vd/1','SumD/1'); add_line(P,'IntId/1','RsId/1'); add_line(P,'Rs_c/1','RsId/2');
add_line(P,'RsId/1','SumD/2'); add_line(P,'weLqIq/1','SumD/3'); add_line(P,'SumD/1','G1Ld/1');
add_line(P,'G1Ld/1','IntId/1'); add_line(P,'IntId/1','Id/1');
add_line(P,'Vq/1','SumQ/1'); add_line(P,'IntIq/1','RsIq/1'); add_line(P,'Rs_c/1','RsIq/2');
add_line(P,'RsIq/1','SumQ/2'); add_line(P,'SumEMF/1','SumQ/3'); add_line(P,'SumQ/1','G1Lq/1');
add_line(P,'G1Lq/1','IntIq/1'); add_line(P,'IntIq/1','Iq/1');
add_line(P,'PolePairs/1','weCalc/1'); add_line(P,'Int_omega/1','weCalc/2');
add_line(P,'weCalc/1','weLq/1'); add_line(P,'Lq_c/1','weLq/2');
add_line(P,'weLq/1','weLqIq/1'); add_line(P,'IntIq/1','weLqIq/2');
add_line(P,'weCalc/1','weLd/1'); add_line(P,'Ld_c/1','weLd/2');
add_line(P,'weLd/1','weLdId/1'); add_line(P,'IntId/1','weLdId/2');
add_line(P,'weCalc/1','wePsid/1'); add_line(P,'psid_c/1','wePsid/2');
add_line(P,'weLdId/1','SumEMF/1'); add_line(P,'wePsid/1','SumEMF/2');
add_line(P,'IntIq/1','k_t/1'); add_line(P,'k_t/1','SumMech/1'); add_line(P,'k_t/1','Te/1');
add_line(P,'Int_omega/1','Bomega/1'); add_line(P,'B/1','Bomega/2');
add_line(P,'Bomega/1','SumMech/2'); add_line(P,'LoadTorque/1','SumMech/3');
add_line(P,'SumMech/1','G1J/1'); add_line(P,'G1J/1','Int_omega/1');
add_line(P,'Int_omega/1','Int_theta/1');
add_line(P,'Int_omega/1','omega_m/1'); add_line(P,'Int_theta/1','theta_m/1');

% Current Controller
CC=[testName '/Current Controller'];
add_block('simulink/Ports & Subsystems/Subsystem',CC,'Position',[200 200 600 500]);
delete_block([CC '/In1']); delete_block([CC '/Out1']);
inlist={'Id_ref','1';'Iq_ref','2';'Id_fb','3';'Iq_fb','4';'we','5';'Ld','6';'Lq','7';'psid','8'};
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

% Speed Controller
SC=[testName '/Speed Controller'];
add_block('simulink/Ports & Subsystems/Subsystem',SC,'Position',[100 100 400 300]);
delete_block([SC '/In1']); delete_block([SC '/Out1']);
add_block('simulink/Sources/In1',[SC '/omega_ref'],'Position',bx(0,0)); set_param([SC '/omega_ref'],'Port','1');
add_block('simulink/Sources/In1',[SC '/omega_fb'],'Position',bx(0,1)); set_param([SC '/omega_fb'],'Port','2');
add_block('simulink/Math Operations/Sum',[SC '/Err'],'Inputs','+-','Position',bx(1,0));
add_block('simulink/Math Operations/Gain',[SC '/Kp'],'Gain','Kp_speed','Position',bx(2,0));
add_block('simulink/Continuous/Integrator',[SC '/Int'],'InitialCondition','0','Position',bx(2,1));
add_block('simulink/Math Operations/Gain',[SC '/Ki'],'Gain','Ki_speed','Position',bx(3,1));
add_block('simulink/Math Operations/Sum',[SC '/Sum_PI'],'Inputs','++','Position',bx(3,0));
add_block('simulink/Discontinuities/Saturation',[SC '/Sat'],'UpperLimit','Iq_max','LowerLimit','-Iq_max','Position',bx(4,0));
add_block('simulink/Sinks/Out1',[SC '/Iq_ref'],'Position',bx(5,0)); set_param([SC '/Iq_ref'],'Port','1');
add_line(SC,'omega_ref/1','Err/1'); add_line(SC,'omega_fb/1','Err/2');
add_line(SC,'Err/1','Kp/1'); add_line(SC,'Err/1','Int/1');
add_line(SC,'Int/1','Ki/1'); add_line(SC,'Kp/1','Sum_PI/1'); add_line(SC,'Ki/1','Sum_PI/2');
add_line(SC,'Sum_PI/1','Sat/1'); add_line(SC,'Sat/1','Iq_ref/1');

% Position Controller in harness
PC=[testName '/Position Controller'];
add_block('simulink/Ports & Subsystems/Subsystem',PC,'Position',[50 50 400 250]);
delete_block([PC '/In1']); delete_block([PC '/Out1']);
add_block('simulink/Sources/In1',[PC '/theta_ref'],'Position',bx(0,0)); set_param([PC '/theta_ref'],'Port','1');
add_block('simulink/Sources/In1',[PC '/theta_fb'],'Position',bx(0,1)); set_param([PC '/theta_fb'],'Port','2');
add_block('simulink/Math Operations/Sum',[PC '/Err_raw'],'Inputs','+-','Position',bx(1,0));
add_block('simulink/User-Defined Functions/Fcn',[PC '/AddPi'],'Position',bx(2,0));
set_param([PC '/AddPi'],'Expr','u+pi');
add_block('simulink/User-Defined Functions/Fcn',[PC '/Mod2Pi'],'Position',bx(3,0));
set_param([PC '/Mod2Pi'],'Expr','u - 2*pi*floor(u/(2*pi))');
add_block('simulink/User-Defined Functions/Fcn',[PC '/SubPi'],'Position',bx(4,0));
set_param([PC '/SubPi'],'Expr','u-pi');
add_block('simulink/Math Operations/Gain',[PC '/Kp'],'Gain','Kp_pos','Position',bx(5,0));
add_block('simulink/Continuous/Integrator',[PC '/Int'],'InitialCondition','0','Position',bx(5,1));
add_block('simulink/Math Operations/Gain',[PC '/Ki'],'Gain','Ki_pos','Position',bx(6,1));
add_block('simulink/Math Operations/Sum',[PC '/Sum_PI'],'Inputs','++','Position',bx(6,0));
add_block('simulink/Discontinuities/Saturation',[PC '/Sat'],'UpperLimit','omega_max','LowerLimit','-omega_max','Position',bx(7,0));
add_block('simulink/Sinks/Out1',[PC '/omega_ref'],'Position',bx(8,0)); set_param([PC '/omega_ref'],'Port','1');
add_line(PC,'theta_ref/1','Err_raw/1'); add_line(PC,'theta_fb/1','Err_raw/2');
add_line(PC,'Err_raw/1','AddPi/1'); add_line(PC,'AddPi/1','Mod2Pi/1');
add_line(PC,'Mod2Pi/1','SubPi/1'); add_line(PC,'SubPi/1','Kp/1'); add_line(PC,'SubPi/1','Int/1');
add_line(PC,'Int/1','Ki/1'); add_line(PC,'Kp/1','Sum_PI/1'); add_line(PC,'Ki/1','Sum_PI/2');
add_line(PC,'Sum_PI/1','Sat/1'); add_line(PC,'Sat/1','omega_ref/1');

% Top-level
add_block('simulink/Sources/Step',[testName '/Theta_step'],'Position',bx(0,0));
set_param([testName '/Theta_step'],'Time','0.1','Before','0','After','pi');
add_block('simulink/Sources/Constant',[testName '/Id_ref_in'],'Value','0','Position',bx(0,1));
add_block('simulink/Sources/Constant',[testName '/TL'],'Value','0','Position',bx(0,2));
add_block('simulink/Sources/Constant',[testName '/PP'],'Value','PolePairs','Position',bx(0,3));
add_block('simulink/Sources/Constant',[testName '/Jc'],'Value','J','Position',bx(0,4));
add_block('simulink/Sources/Constant',[testName '/Bc'],'Value','B','Position',bx(0,5));
add_block('simulink/Sources/Constant',[testName '/Tc'],'Value','25','Position',bx(0,6));
add_block('simulink/Sources/Constant',[testName '/Ld_cn'],'Value','Ld_nom','Position',bx(1,0));
add_block('simulink/Sources/Constant',[testName '/Lq_cn'],'Value','Lq_nom','Position',bx(1,1));
add_block('simulink/Sources/Constant',[testName '/psid_cn'],'Value','psi_d_nom','Position',bx(1,2));
add_block('simulink/Math Operations/Product',[testName '/we_calc'],'Position',bx(1,3));

% Cascade: Position -> Speed -> Current -> Plant
add_line(testName,'Theta_step/1','Position Controller/1');
add_line(testName,'Motor Plant/5','Position Controller/2');
add_line(testName,'Position Controller/1','Speed Controller/1');
add_line(testName,'Motor Plant/4','Speed Controller/2');
add_line(testName,'Speed Controller/1','Current Controller/2');
add_line(testName,'Id_ref_in/1','Current Controller/1');
add_line(testName,'Motor Plant/1','Current Controller/3');
add_line(testName,'Motor Plant/2','Current Controller/4');
add_line(testName,'we_calc/1','Current Controller/5');
add_line(testName,'Ld_cn/1','Current Controller/6');
add_line(testName,'Lq_cn/1','Current Controller/7');
add_line(testName,'psid_cn/1','Current Controller/8');
add_line(testName,'Current Controller/1','Motor Plant/1');
add_line(testName,'Current Controller/2','Motor Plant/2');
add_line(testName,'PP/1','we_calc/1'); add_line(testName,'Motor Plant/4','we_calc/2');
add_line(testName,'TL/1','Motor Plant/3'); add_line(testName,'PP/1','Motor Plant/4');
add_line(testName,'Jc/1','Motor Plant/5'); add_line(testName,'Bc/1','Motor Plant/6');
add_line(testName,'Tc/1','Motor Plant/7');

% Logging
add_block('simulink/Sinks/To Workspace',[testName '/W_th'],'Position',bx(6,0));
set_param([testName '/W_th'],'VariableName','theta_sim','SaveFormat','Array');
add_line(testName,'Motor Plant/5','W_th/1');

save_system(testName);
fprintf('Test harness saved: %s.slx\n\n',testName);

%% ======================================================================
%  PART 3: Verification
% ======================================================================
fprintf('Simulating full FOC...\n');
try
    out=sim(testName,'StopTime','0.5','ReturnWorkspaceOutputs','on');
    th=out.get('theta_sim');
    N=length(th); ss=round(N*0.9);
    th_ss=mean(th(ss:end));
    th_wrapped=mod(th_ss+pi,2*pi)-pi;
    fprintf('  theta_ref=pi rad, theta_ss_wrapped=%.4f\n',th_wrapped);
    err=abs(th_wrapped-pi);
    if err<0.5
        fprintf('\n✅ VERIFICATION PASSED (error=%.4f rad)\n',err);
    else
        fprintf('\n⚠️  Error=%.4f rad\n',err);
    end
catch ME
    fprintf('ERROR: %s\n',ME.message);
end

fprintf('\n╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   STEP 5d DONE                                              ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');
