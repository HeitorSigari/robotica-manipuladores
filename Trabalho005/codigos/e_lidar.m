% e_lidar.m - Alternativa E: lidar (tempo de voo e diferenca de fase)
clc; clear; close all;                            % limpa tela, variaveis e figuras

c = 299792458;                                    % velocidade da luz [m/s]

% ---- E.1: resolucao temporal (tempo de voo) ----
D = [1 1e-3];                                     % distancias a distinguir: 1 m e 1 mm [m]
t_ida = D/c;                                      % tempo SO DE IDA (valor dos slides) [s]
t_ida_volta = 2*D/c;                              % tempo de ida e volta (correto) [s]
printf('Apenas ida:      1 m -> %.3e s | 1 mm -> %.3e s\n', t_ida(1), t_ida(2));
printf('Ida e volta:     1 m -> %.3e s | 1 mm -> %.3e s\n', t_ida_volta(1), t_ida_volta(2));

% ---- E.2: lidar por diferenca de fase ----
lambda = 60;                                      % comprimento de onda da luz modulada [m]
dist = @(th_graus) lambda*deg2rad(th_graus)/(4*pi);   % D = lambda*theta/(4*pi), theta em graus
printf('(a) D(90 graus) = %.2f m | Dmax = lambda/2 = %.1f m\n', dist(90), lambda/2);

D_real = 35;                                      % distancia real do obstaculo [m]
theta_real = 4*pi*D_real/lambda;                  % diferenca de fase real [rad]
theta_graus = rad2deg(theta_real);                % mesma fase em graus
theta_med = mod(theta_graus, 360);                % fase medida: so se enxerga o resto da divisao por 360
printf('(b) Fase real = %.1f graus -> fase medida = %.1f graus -> D informada = %.2f m\n', ...
       theta_graus, theta_med, dist(theta_med));

dD = dist(0.1);                                   % menor variacao de distancia para delta_theta = 0,1 grau [m]
printf('(c) Menor variacao detectavel = %.5f m = %.2f mm\n', dD, 1e3*dD);
