function Q = apply_settings(P,kind,g)
Q=P;
switch kind
    case 'static_ny', Q.static=g;
    case 'astatic_ny', Q.astatic=g;
    case 'static_pitch', Q.pitch_static=g;
    case 'astatic_pitch', Q.pitch_astatic=g;
end
end
