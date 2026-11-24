% b_acuracia_precisao.m - Alternativa B: alcance, resolucao, acuracia e precisao
clc; clear; close all;                            % limpa tela, variaveis e figuras

v = 2.000;                                        % distancia real do alvo estacionario [m]
A = [1.95 1.97 1.96 1.98 1.96 1.97 1.95 1.96];    % 8 leituras do sensor A [m]
B = [2.10 2.11 2.10 2.09 2.10 2.11 2.10 2.10];    % 8 leituras do sensor B [m]

% ---------- B.1: media, acuracia e desvio-padrao amostral ----------
mA = mean(A);                                     % media das leituras de A [m]
mB = mean(B);                                     % media das leituras de B [m]
accA = 1 - abs(mA - v)/v;                         % acuracia de A = 1 - |m - v|/v
accB = 1 - abs(mB - v)/v;                         % acuracia de B
SQA = sum((A - mA).^2);                           % soma dos quadrados dos desvios de A [m^2]
SQB = sum((B - mB).^2);                           % soma dos quadrados dos desvios de B [m^2]
sA = sqrt(SQA/(numel(A) - 1));                    % desvio-padrao amostral de A (divide por n-1) [m]
sB = sqrt(SQB/(numel(B) - 1));                    % desvio-padrao amostral de B [m]
printf('Sensor A: media = %.5f m | acuracia = %.3f %% | soma(xi-m)^2 = %.6f | desvio = %.4f m\n', mA, 100*accA, SQA, sA);
printf('Sensor B: media = %.5f m | acuracia = %.3f %% | soma(xi-m)^2 = %.6f | desvio = %.4f m\n', mB, 100*accB, SQB, sB);

% ---------- B.3: calibracao por compensacao de offset do sensor B ----------
offset = mB - v;                                  % erro sistematico medio de B [m]
Bc = B - offset;                                  % leituras de B apos subtrair o offset [m]
mBc = mean(Bc);                                   % nova media [m]
accBc = 1 - abs(mBc - v)/v;                       % nova acuracia
sBc = std(Bc);                                    % novo desvio-padrao (nao muda: subtrair constante)
printf('Offset de B = %.5f m\n', offset);
printf('B calibrado: media = %.5f m | acuracia = %.3f %% | desvio = %.4f m\n', mBc, 100*accBc, sBc);

% ---------- B.4: distancia percorrida entre duas leituras ----------
f = 10;                                           % frequencia de atualizacao do sensor [Hz]
vr = 0.5;                                         % velocidade do robo [m/s]
dt = 1/f;                                         % tempo entre leituras [s]
d = vr*dt;                                        % distancia percorrida entre leituras [m]
printf('Entre duas leituras: dt = %.2f s -> d = %.2f m\n', dt, d);
