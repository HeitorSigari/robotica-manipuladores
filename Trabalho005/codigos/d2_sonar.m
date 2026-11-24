% d2_sonar.m - Alternativa D.2: erro de distancia por temperatura errada no firmware
clc; clear; close all;                            % limpa tela, variaveis e figuras

tv = 18.4e-3;                                     % tempo de voo medido [s]
c_fw = 343.4;                                     % velocidade do som assumida pelo firmware (20 graus C) [m/s]
c_real = 349.2;                                   % velocidade do som real a 30 graus C (tabela) [m/s]
d_inf = c_fw*tv/2;                                % distancia informada: divide por 2 (ida e volta) [m]
d_real = c_real*tv/2;                             % distancia real [m]
e_abs = abs(d_inf - d_real);                      % erro absoluto [m]
e_rel = e_abs/d_real;                             % erro relativo
printf('Distancia informada = %.4f m\n', d_inf);
printf('Distancia real      = %.4f m\n', d_real);
printf('Erro absoluto       = %.4f m\n', e_abs);
printf('Erro relativo       = %.4f = %.2f %%\n', e_rel, 100*e_rel);
