function p = static_characteristic(P, Kn, Kw)
% Characteristic polynomial for the static ny loop including the actuator.
k = P.c6_over_g;
aircraft_den = [1, P.c1+P.c4+P.c5, P.c2+P.c1*P.c4];
ny_num = k*[P.c9, P.c9*(P.c1+P.c5), -P.c3*P.c4+P.c2*P.c9];
omega_num = [-(P.c3-P.c5*P.c9), -(P.c3*P.c4-P.c2*P.c9)];
feedback_num = Kn*ny_num + Kw*[0, omega_num];
if P.actuator.dynamic
    actuator_den = [1, 2*P.actuator.zeta*P.actuator.Omega, ...
        P.actuator.Omega^2];
    actuator_num = P.actuator.Omega^2;
else
    actuator_den = 1;
    actuator_num = 1;
end
p = conv(aircraft_den,actuator_den);
q = actuator_num*feedback_num;
p(end-numel(q)+1:end) = p(end-numel(q)+1:end) - q;
end
