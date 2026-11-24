% d3_anel_sonar.m - Alternativa D.3: anel de 16 sonares (alcance x taxa de atualizacao)
clc; clear; close all;                            % limpa tela, variaveis e figuras
if ~exist('figuras','dir'), mkdir('figuras'); end % cria a pasta de figuras se nao existir

c = 343;                                          % velocidade do som [m/s]
N = 16;                                           % numero de sonares do anel (disparam um por vez)

% ---- (a) anel a 10 Hz ----
f_anel = 10;                                      % taxa de atualizacao desejada [Hz]
T_ciclo = 1/f_anel;                               % duracao de um ciclo completo do anel [s]
t_max = T_ciclo/N;                                % tempo de voo maximo por sonar [s]
D_max = c*t_max/2;                                % alcance maximo (ida e volta) [m]
printf('(a) Ciclo = %.0f ms | tempo de voo max por sonar = %.2f ms | alcance max = %.3f m\n', ...
       1e3*T_ciclo, 1e3*t_max, D_max);

% ---- (b) tempo usual de 40 ms por sonar ----
t_s = 40e-3;                                      % tempo por sonar [s]
D_s = c*t_s/2;                                    % alcance correspondente [m]
T_c = N*t_s;                                      % ciclo completo do anel [s]
f_c = 1/T_c;                                      % taxa de atualizacao do anel [Hz]
printf('(b) Alcance = %.2f m | ciclo = %.2f s | taxa = %.3f Hz\n', D_s, T_c, f_c);

% ---- (c) compromisso alcance x taxa: f = c/(2*N*Dmax) ----
Dm = linspace(0.5, 8, 200);                       % alcances maximos desejados [m]
f = c./(2*N*Dm);                                  % taxa de atualizacao possivel para cada alcance [Hz]
figure('Position', [100 100 700 450]);            % abre a figura
plot(Dm, f, 'b-', 'LineWidth', 1.5); hold on;     % curva do compromisso
plot(D_max, f_anel, 'ro', 'MarkerFaceColor', 'r');% ponto do item (a)
plot(D_s, f_c, 'ks', 'MarkerFaceColor', 'k');     % ponto do item (b)
grid on;                                          % grade
xlabel('Alcance maximo D_{max} (m)');             % eixo x identificado
ylabel('Taxa de atualizacao do anel (Hz)');       % eixo y identificado
title('Anel de 16 sonares: alcance x taxa de atualizacao');   % titulo
legend('f = c/(2 N D_{max})', 'Item (a): 10 Hz', 'Item (b): 40 ms/sonar');   % legenda
print('-dpng', '-r150', 'figuras/sonar_compromisso.png');    % salva a figura
