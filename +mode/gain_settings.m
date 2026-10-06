function [g,keys] = gain_settings(P,kind)
% Одинаковая структура настроек для всех контуров.
switch kind
    case 'static_ny', src=P.static;
    case 'astatic_ny', src=P.astatic;
    case 'static_pitch', src=P.pitch_static;
    case 'astatic_pitch', src=P.pitch_astatic;
    otherwise, error('Неизвестная система.');
end
g=struct('Kn',0,'Kw',0,'Ki',0,'Ktheta',0);
f=fieldnames(src);
for i=1:numel(f), g.(f{i})=src.(f{i}); end
keys={'Kn','Kw'};
if startsWith(kind,'astatic_'), keys{end+1}='Ki'; end
if endsWith(kind,'_pitch'), keys{end+1}='Ktheta'; end
end
