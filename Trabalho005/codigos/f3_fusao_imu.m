% f3_fusao_imu.m - Alternativa F.3: fusao giroscopio + acelerometro (filtro complementar)
clc; clear; close all;                            % limpa tela, variaveis e figuras
if ~exist('figuras','dir'), mkdir('figuras'); end % cria a pasta de figuras se nao existir
randn('state', 1);                                % semente fixa: resultados reproduziveis

dt = 0.01;                                        % passo de tempo [s]
t = 0:dt:60;                                      % vetor de tempo de 0 a 60 s [s]
N = numel(t);                                     % numero de amostras
th = 30*sin(2*pi*0.2*t);                          % angulo real [graus]
om = 30*2*pi*0.2*cos(2*pi*0.2*t);                 % velocidade angular real = derivada de th [graus/s]

bias = 0.5;                                       % bias do giroscopio [graus/s]
sg = 0.2;                                         % desvio-padrao do ruido do giroscopio [graus/s]
sa = 2;                                           % desvio-padrao do ruido do acelerometro [graus]
w = om + bias + sg*randn(1,N);                    % medida do giroscopio [graus/s]
ac = th + sa*randn(1,N);                          % medida de angulo do acelerometro [graus]

rmse = @(est) sqrt(mean((est - th).^2));          % raiz do erro quadratico medio em relacao ao angulo real

% ---------- (1) integrando apenas o giroscopio ----------
th_g = th(1) + [0 cumsum(w(2:end))*dt];           % integral numerica de w a partir do angulo inicial real [graus]

% ---------- (2) usando apenas o acelerometro ----------
th_a = ac;                                        % o proprio angulo medido [graus]

% ---------- (3) filtro complementar ----------
function est = complementar(w, ac, dt, alfa)
  est = zeros(size(w));                           % vetor da estimativa
  est(1) = ac(1);                                 % inicializa com a primeira medida do acelerometro
  for k = 2:numel(w)                              % recursao do filtro
    est(k) = alfa*(est(k-1) + w(k)*dt) + (1-alfa)*ac(k);   % theta[k] = a(theta[k-1]+w[k]dt) + (1-a)theta_acel[k]
  end
end
th_c = complementar(w, ac, dt, 0.98);             % filtro com alfa = 0,98

printf('RMSE giroscopio    = %.3f graus\n', rmse(th_g));
printf('RMSE acelerometro  = %.3f graus\n', rmse(th_a));
printf('RMSE complementar  = %.3f graus (alfa = 0,98)\n', rmse(th_c));

% ---------- graficos das tres estimativas ----------
est = {th_g, th_a, th_c};                         % estimativas
nomes = {'Giroscopio (integracao)', 'Acelerometro', 'Filtro complementar (alfa = 0,98)'};   % titulos
cores = {'r', [0.4 0.4 0.4], 'b'};                % cores das estimativas
figure('Position', [100 100 760 760]);            % abre a figura
for i = 1:3                                       % um painel por estimativa
  subplot(3,1,i);                                 % painel i
  plot(t, th, 'k-', 'LineWidth', 1.5); hold on;   % angulo real em preto
  plot(t, est{i}, '-', 'Color', cores{i});        % estimativa
  grid on; ylabel('Angulo (graus)');              % grade e eixo y
  title(sprintf('%s - RMSE = %.2f graus', nomes{i}, rmse(est{i})));   % titulo com o RMSE
  legend('Real', 'Estimado', 'Location', 'northeast');   % legenda
  if i == 3, xlabel('Tempo (s)'); end             % eixo x no ultimo painel
end
print('-dpng', '-r150', 'figuras/imu_tres_estimativas.png');   % salva a figura

% ---------- influencia de alfa ----------
alfas = [0.90 0.98 0.995];                        % valores de alfa pedidos
figure('Position', [100 100 760 520]);            % abre a figura
cs = {'r', 'b', [0 0.6 0]};                       % cores das tres curvas
for i = 1:numel(alfas)                            % para cada alfa
  e = complementar(w, ac, dt, alfas(i));          % estimativa com esse alfa
  plot(t, e - th, '-', 'Color', cs{i}); hold on;  % desenha o ERRO de estimacao (estimado - real) [graus]
  printf('alfa = %.3f | tau = %.3f s | RMSE = %.3f graus\n', alfas(i), alfas(i)*dt/(1-alfas(i)), rmse(e));
end
grid on; xlabel('Tempo (s)'); ylabel('Erro de estimacao (graus)');   % grade e eixos
legend('\alpha = 0,90', '\alpha = 0,98', '\alpha = 0,995', 'Location', 'northeast');   % legenda
title('Erro do filtro complementar para diferentes valores de \alpha');   % titulo
print('-dpng', '-r150', 'figuras/imu_alfas.png');      % salva a figura

% ---------- RMSE em funcao de alfa ----------
av = [0.5 0.7 0.8 0.9 0.95 0.98 0.99 0.995 0.998];% varredura de alfa
rv = zeros(size(av));                             % RMSE para cada alfa
for i = 1:numel(av)                               % calcula o RMSE de cada alfa
  rv(i) = rmse(complementar(w, ac, dt, av(i)));   % RMSE
end
figure('Position', [100 100 700 420]);            % abre a figura
semilogy(av, rv, 'bo-', 'LineWidth', 1.5); grid on;   % RMSE x alfa
xlabel('\alpha'); ylabel('RMSE (graus)');         % eixos identificados
title('RMSE do filtro complementar em funcao de \alpha');   % titulo
print('-dpng', '-r150', 'figuras/imu_rmse_alfa.png');  % salva a figura
