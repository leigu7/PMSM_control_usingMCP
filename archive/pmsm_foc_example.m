% pmsm_foc_example  Example simulation of a surface-mounted PMSM with FOC.

clearvars; close all;

data = simulate_surface_pmsm_foc('StopTime', 0.5, 'Temperature', 25, ...
    'PositionReference', @(t) 2*pi*1.0*t, 'SampleTime', 1e-4);

figure('Name','PMSM FOC Simulation','Units','normalized','Position',[0.1 0.1 0.8 0.7]);
subplot(3,2,1);
plot(data.time, data.theta_m);
hold on;
plot(data.time, 2*pi*1.0*data.time, '--');
xlabel('Time (s)'); ylabel('Rotor angle (rad)');
title('Rotor Position');
legend('Measured','Reference');
grid on;

subplot(3,2,2);
plot(data.time, data.omega_m);
hold on;
plot(data.time, data.omega_ref, '--');
xlabel('Time (s)'); ylabel('Speed (rad/s)');
title('Electrical Speed');
legend('Motor','Reference');
grid on;

subplot(3,2,3);
plot(data.time, data.id);
hold on;
plot(data.time, zeros(size(data.time)), '--');
xlabel('Time (s)'); ylabel('Id (A)');
title('D-axis Current');
legend('Id','Reference');
grid on;

subplot(3,2,4);
plot(data.time, data.iq);
hold on;
plot(data.time, data.iq_ref, '--');
xlabel('Time (s)'); ylabel('Iq (A)');
title('Q-axis Current');
legend('Iq','Reference');
grid on;

subplot(3,2,5);
plot(data.time, data.v_d);
plot(data.time, data.v_q);
xlabel('Time (s)'); ylabel('Voltage (V)');
title('dq Voltage Commands');
legend('V_d','V_q');
grid on;

subplot(3,2,6);
plot(data.time, data.torque);
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('Electromagnetic Torque');
grid on;

sgtitle('Surface-mounted PMSM FOC Closed-loop Simulation');
