% f1_acelerometro.m - Alternativa F.1: acelerometro como sistema massa-mola-amortecedor
clc; clear; close all;                            % limpa tela, variaveis e figuras
pkg load control;                                 % carrega o pacote de controle (tf, step, bode)
if ~exist('figuras','dir'), mkdir('figuras'); end % cria a pasta de figuras se nao existir

m = 1e-6;                                         % massa de prova [kg]
k = 1e5;                                          % constante da mola [N/m]
c = 0.443;                                        % coeficiente de amortecimento [N.s/m]
g = 9.81;                                         % aceleracao da gravidade [m/s^2]

G = tf(1, [m c k]);                               % funcao de transferencia X(s)/F(s) = 1/(m s^2 + c s + k)
wn = sqrt(k/m);                                   % frequencia natural [rad/s]
fn = wn/(2*pi);                                   % frequencia natural [Hz]
zeta = c/(2*sqrt(k*m));                           % coeficiente de amortecimento (adimensional)
S = m/k;                                          % sensibilidade estatica [m por (m/s^2)]
x1g = S*g;                                        % deslocamento da massa para 1 g [m]
printf('wn = %.4e rad/s | fn = %.2f Hz | zeta = %.4f\n', wn, fn, zeta);
printf('Sensibilidade m/k = %.3e m/(m/s^2) | deslocamento para 1 g = %.3e m\n', S, x1g);

% ---------- resposta ao degrau ----------
[y, t] = step(G, linspace(0, 100e-6, 2000));      % resposta ao degrau de forca unitaria em 100 microssegundos
y = y(:); t = t(:);                               % garante vetores coluna
yf = 1/k;                                         % valor final da resposta (ganho DC = 1/k) [m]
Mp = max(y)/yf - 1;                               % sobressinal relativo
tp = t(find(y == max(y), 1));                     % instante do pico [s]
ts = t(find(abs(y/yf - 1) > 0.02, 1, 'last') + 1);% tempo de acomodacao (criterio de 2 %) [s]
printf('Sobressinal = %.2f %% | tempo de pico = %.2f us | acomodacao (2 %%) = %.2f us\n', 100*Mp, 1e6*tp, 1e6*ts);
figure('Position', [100 100 700 420]);            % abre a figura do degrau
plot(1e6*t, y/yf, 'b-', 'LineWidth', 1.5); hold on;   % resposta normalizada pelo valor final
plot(1e6*[t(1) t(end)], [1 1], 'k--');            % linha do valor final
grid on;                                          % grade
xlabel('Tempo (\mu s)'); ylabel('x(t) / x_{final}');  % eixos identificados
title('Resposta ao degrau do acelerometro (normalizada)');    % titulo
print('-dpng', '-r150', 'figuras/acel_degrau.png');            % salva a figura

% ---------- diagrama de Bode ----------
f = logspace(3, 6, 3000);                         % frequencias de 1 kHz a 1 MHz [Hz]
[mag, fase] = bode(G, 2*pi*f);                    % modulo (absoluto) e fase (graus) na frequencia em rad/s
mag = squeeze(mag); fase = squeeze(fase);         % remove dimensoes extras
magdB = 20*log10(mag);                            % modulo em dB
i3 = find(mag < mag(1)/sqrt(2), 1);               % primeira frequencia com queda de 3 dB em relacao ao ganho DC
f3dB = f(i3);                                     % banda passante (-3 dB) [Hz]
printf('Banda passante (-3 dB) = %.2f kHz\n', f3dB/1e3);
figure('Position', [100 100 700 520]);            % abre a figura do Bode
subplot(2,1,1);                                   % painel de modulo
semilogx(f, magdB, 'b-', 'LineWidth', 1.5); hold on;  % modulo em dB x frequencia
plot([50e3 50e3], [min(magdB) max(magdB)], 'r--');    % marca os 50 kHz dos slides
plot([f3dB f3dB], [min(magdB) max(magdB)], 'k:');     % marca a banda de -3 dB
grid on; ylabel('Modulo (dB)');                   % grade e eixo y
title('Diagrama de Bode de X(s)/F(s)');           % titulo
subplot(2,1,2);                                   % painel de fase
semilogx(f, fase, 'b-', 'LineWidth', 1.5); grid on;   % fase x frequencia
xlabel('Frequencia (Hz)'); ylabel('Fase (graus)');    % eixos identificados
print('-dpng', '-r150', 'figuras/acel_bode.png');      % salva a figura
