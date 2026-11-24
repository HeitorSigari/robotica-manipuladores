% c1_resolucao.m - Alternativa C.1: contagens e resolucao angular de encoders
clc; clear; close all;                            % limpa tela, variaveis e figuras

ticks = [2048 64];                                % ticks por revolucao dos dois encoders
for i = 1:numel(ticks)                            % repete o calculo para cada encoder
  n = ticks(i);                                   % ticks por revolucao do encoder atual
  cont_reg = n;                                   % modo regular: 1 contagem por tick
  cont_quad = 4*n;                                % quadratura: 4 contagens por tick (bordas de A e B)
  res_reg = 360/cont_reg;                         % resolucao angular no modo regular [graus]
  res_quad = 360/cont_quad;                       % resolucao angular em quadratura [graus]
  printf('Encoder %4d ticks: regular = %5d contagens, %.4f graus | quadratura = %5d contagens, %.4f graus\n', ...
         n, cont_reg, res_reg, cont_quad, res_quad);
end
