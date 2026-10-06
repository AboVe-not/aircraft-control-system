function sys = closed_loop(P, kind)
% Exact linear block interconnection. Inputs are reference, gust, moment.
% Outputs are ny, omega, theta, delta. Moment means M_z/I_z in deg/s^2.
[A,B,C,D] = lr1.plant_matrices(P);
aircraft = ss(A,B,C,D);
aircraft.InputName = {'delta','gust','moment'};
aircraft.OutputName = {'ny','omega','theta'};

if P.actuator.dynamic
    actuator = tf(P.actuator.Omega^2, ...
        [1,2*P.actuator.zeta*P.actuator.Omega,P.actuator.Omega^2]);
else
    actuator = tf(1);
end
actuator.InputName = 'command';
actuator.OutputName = 'delta';

if strcmp(kind,'static_pitch') && isfield(P.pitch_static,'Kn')
    gains = P.pitch_static;
elseif strcmp(kind,'astatic_pitch') && isfield(P.pitch_astatic,'Kn')
    gains = P.pitch_astatic;
elseif startsWith(kind,'static_')
    gains = P.static;
else
    gains = P.astatic;
end
Kn = tf(gains.Kn); Kn.InputName='ny_error'; Kn.OutputName='ny_part';
Kw = tf(gains.Kw); Kw.InputName='omega'; Kw.OutputName='rate_part';

if endsWith(kind,'_pitch')
    if startsWith(kind,'static_')
        theta_gain = P.pitch_static.Ktheta;
    else
        theta_gain = P.pitch_astatic.Ktheta;
    end
    Ktheta = tf(theta_gain);
    Ktheta.InputName = 'theta_error'; Ktheta.OutputName = 'ny_ref';
    outer = {Ktheta,sumblk('theta_error = reference - theta')};
    inner_ref = 'ny_ref';
else
    outer = {};
    inner_ref = 'reference';
end

if startsWith(kind,'static_')
    err = sumblk(['ny_error = ny - ' inner_ref]);
    command = sumblk('command = ny_part + rate_part');
    parts = {aircraft,actuator,Kn,Kw,outer{:},err,command};
elseif startsWith(kind,'astatic_')
    % Figure 2: integral(error) plus direct ny and omega feedback.
    Ki = tf(gains.Ki,[1 0]);
    Ki.InputName = 'ny_error'; Ki.OutputName = 'integral_part';
    Kn.InputName = 'ny';
    err = sumblk(['ny_error = ny - ' inner_ref]);
    command = sumblk('command = integral_part + ny_part + rate_part');
    parts = {aircraft,actuator,Kn,Kw,Ki,outer{:},err,command};
else
    error('Unknown circuit: %s',kind);
end

sys = connect(parts{:}, {'reference','gust','moment'}, ...
    {'ny','omega','theta','delta'});
sys.InputName = {'reference','gust','moment'};
sys.OutputName = {'ny','omega','theta','delta'};
end
