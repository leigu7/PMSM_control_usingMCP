function data = simulate_surface_pmsm_foc(varargin)
% simulate_surface_pmsm_foc  Simulate a surface-mounted PMSM with FOC loops.
%
%   data = simulate_surface_pmsm_foc() runs a default 0.5-second closed-loop
%   simulation with position feedback, speed loop, and inner dq current loops.
%
%   data = simulate_surface_pmsm_foc('StopTime', 0.3, 'Temperature', 25, ...)
%   uses optional name-value parameters.

    p = inputParser;
    addParameter(p, 'StopTime', 0.5, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'Temperature', 25, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'PositionReference', @(t) 2*pi*1.0*t, @(x) isa(x,'function_handle'));
    addParameter(p, 'LoadTorque', 0.0, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'SampleTime', 1e-4, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parse(p, varargin{:});

    Tstop = p.Results.StopTime;
    temperature = p.Results.Temperature;
    positionRefFcn = p.Results.PositionReference;
    Tl = p.Results.LoadTorque;
    dt = p.Results.SampleTime;

    motor = SurfaceMountedPMSM();
    time = 0:dt:Tstop;
    N = numel(time);

    % Controller gains
    Kp_pos = 5.0;
    Ki_pos = 200.0;
    Kp_speed = 0.15;
    Ki_speed = 1.5;
    Kp_id = 40.0;
    Ki_id = 800.0;
    Kp_iq = 40.0;
    Ki_iq = 800.0;

    % Preallocate
    id = zeros(1, N);
    iq = zeros(1, N);
    omega_m = zeros(1, N);
    theta_m = zeros(1, N);
    v_d = zeros(1, N);
    v_q = zeros(1, N);
    omega_ref = zeros(1, N);
    iq_ref = zeros(1, N);
    pos_error = zeros(1, N);
    speed_error = zeros(1, N);
    id_error = zeros(1, N);
    iq_error = zeros(1, N);
    torque = zeros(1, N);
    Rs = motor.interpRs(temperature);

    integrator_pos = 0.0;
    integrator_speed = 0.0;
    integrator_id = 0.0;
    integrator_iq = 0.0;

    % Start from zero rotor angle and speed
    % Use three-loop FOC: position -> speed -> current.
    for k = 1:N
        t = time(k);
        theta_ref = positionRefFcn(t);
        pos_error(k) = wrapToPi(theta_ref - theta_m(k));
        integrator_pos = integrator_pos + pos_error(k) * dt;
        omega_ref(k) = Kp_pos * pos_error(k) + Ki_pos * integrator_pos;

        speed_error(k) = omega_ref(k) - omega_m(k);
        integrator_speed = integrator_speed + speed_error(k) * dt;
        iq_ref(k) = max(min(Kp_speed * speed_error(k) + Ki_speed * integrator_speed, 120), -120);

        id_ref = 0.0;
        id_error(k) = id_ref - id(k);
        iq_error(k) = iq_ref(k) - iq(k);

        integrator_id = integrator_id + id_error(k) * dt;
        integrator_iq = integrator_iq + iq_error(k) * dt;

        [Ld, Lq, psi_d, ~] = motor.lookup(id(k), iq(k));
        omega_e = motor.PolePairs * omega_m(k);

        v_d(k) = Kp_id * id_error(k) + Ki_id * integrator_id + omega_e * Lq * iq(k);
        v_q(k) = Kp_iq * iq_error(k) + Ki_iq * integrator_iq - omega_e * Ld * id(k) - omega_e * psi_d;

        did = (v_d(k) - Rs * id(k) + omega_e * Lq * iq(k)) / Ld;
        diq = (v_q(k) - Rs * iq(k) - omega_e * Ld * id(k) - omega_e * psi_d) / Lq;
        Te = motor.torque(id(k), iq(k));
        torque(k) = Te;
        domega = (Te - Tl - motor.B * omega_m(k)) / motor.J;
        dtheta = omega_m(k);

        if k < N
            id(k+1) = id(k) + did * dt;
            iq(k+1) = iq(k) + diq * dt;
            omega_m(k+1) = omega_m(k) + domega * dt;
            theta_m(k+1) = theta_m(k) + dtheta * dt;
        end
    end

    data = struct();
    data.time = time;
    data.id = id;
    data.iq = iq;
    data.omega_m = omega_m;
    data.theta_m = theta_m;
    data.v_d = v_d;
    data.v_q = v_q;
    data.iq_ref = iq_ref;
    data.omega_ref = omega_ref;
    data.pos_error = pos_error;
    data.speed_error = speed_error;
    data.torque = torque;
    data.Rs = Rs;
    data.motor = motor;
end

function y = wrapToPi(x)
    y = mod(x + pi, 2*pi) - pi;
end
