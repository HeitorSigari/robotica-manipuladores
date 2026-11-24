% f2_inclinacao.m - Alternativa F.2: inclinacao medida por um acelerometro de tres eixos
clc; clear; close all;                            % limpa tela, variaveis e figuras

g = 9.81;                                         % aceleracao da gravidade [m/s^2]

% ---- (a) repouso e inclinado ----
ax = 0.500; ay = 0; az = 0.866;                   % leituras em unidades de g
theta_a = atan2d(ax, az);                         % angulo em torno do eixo y [graus]
printf('(a) Inclinacao = %.3f graus\n', theta_a);

% ---- (b) robo nivelado acelerando a 2 m/s^2 ----
a_x = 2;                                          % aceleracao para frente [m/s^2]
ax_b = a_x/g;                                     % componente x lida (em g)
az_b = 1;                                         % componente z lida (em g): so a gravidade
theta_b = atan2d(ax_b, az_b);                     % inclinacao "indicada" [graus]
printf('(b) ax = %.4f g | inclinacao indicada = %.2f graus (real = 0)\n', ax_b, theta_b);
