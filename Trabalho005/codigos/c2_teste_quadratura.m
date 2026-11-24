% c2_teste_quadratura.m - Alternativa C.2: teste do decodificador em quadratura
clc; clear; close all;                            % limpa tela, variaveis e figuras

est = [1 2 3 4 1 2 1 4 3];                        % sequencia de estados: S1,S2,S3,S4,S1,S2,S1,S4,S3
Aest = [1 1 0 0];                                 % nivel de A em S1,S2,S3,S4 (S1=(1,0) S2=(1,1) S3=(0,1) S4=(0,0))
Best = [0 1 1 0];                                 % nivel de B em S1,S2,S3,S4
A = Aest(est);                                    % amostras do canal A ao longo da sequencia
B = Best(est);                                    % amostras do canal B ao longo da sequencia
[cnt, hist] = quad_decode(A, B);                  % decodifica: contagem liquida e historico
ang = cnt*360/(4*64);                             % angulo = contagens x resolucao (64 ticks, quadratura) [graus]
disp(['Historico da contagem: ' num2str(hist)]);  % mostra a evolucao da contagem
printf('Contagem liquida = %d | angulo = %.4f graus\n', cnt, ang);
