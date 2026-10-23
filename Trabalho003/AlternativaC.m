% ======================================================
% Robô planar 2GL - Cinemática inversa em forma fechada
% ======================================================
clc; clear; close all;          % limpa tela, variáveis e figuras

L1 = 1; L2 = 1;                 % comprimentos dos elos 1 e 2

% ---------- Escolha do caminho ----------
n_caminho = input(['Caminho:\n 1) Reta\n 2) Semi-círculo\n 3) Cúbica\n> ']);  % lê a opção do usuário

switch n_caminho                % seleciona o caminho conforme a opção
  case 1                        % Caminho 1: reta
    Caminho.x = -1:0.05:1;                          % x de -1 a 1, passo 0,05
    Caminho.y = tand(30) * Caminho.x + 0.5;         % y = tan(30°)·x + 0,5 (equação da reta)
  case 2                        % Caminho 2: semi-círculo
    Caminho.x = 0.5 + 1.2 * cosd(0:2:180);          % centro x = 0,5, raio 1,2, ângulo de 0° a 180°
    Caminho.y = 1.2 * sind(0:2:180);                % y do semi-círculo
  case 3                        % Caminho 3: cúbica
    Caminho.x = -1:0.05:1;                          % x de -1 a 1, passo 0,05
    Caminho.y = Caminho.x.^3 + 0.5;                 % y = x³ + 0,5 (.^ opera elemento a elemento)
  otherwise
    error('Opção inválida');    % interrompe se a opção não existe
endswitch

% ---------- Matrizes de resultado ----------
N = length(Caminho.x);          % número de pontos do caminho
TH1 = NaN(2,N);                 % ângulos da junta 1 [graus]: linha 1 = cotovelo cima, linha 2 = baixo
TH2 = NaN(2,N);                 % ângulos da junta 2 [graus]
% Nan incializa a matriz com pontos sem solução ("Not a Number"), que mais tarde serão preenchidos.
% Cada coluna contém os ângulos, no ponto em questão, para cotovelo de cima e para o cotovelo de baixo.

% ---------- Cinemática inversa ----------
for k = 1:N                     % percorre cada ponto do caminho
  x = Caminho.x(k);  y = Caminho.y(k);                    % ponto desejado para o punho
  c2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);            % cálculo do cos(theta2) encontrado na cinemática inversa
  if abs(c2) > 1, continue; end                           % -1 <= cos(theta2) <= 1? Se não, fora do workspace: fica NaN

  sinais = [-1, 1];                                       % -1 = cotovelo para cima, +1 = para baixo (sentido geométrico)
  for i = 1:2                                             % laço interno que calcula as duas configurações
    s2 = sinais(i) * sqrt(1 - c2^2);                      % sen(theta2) = ±raiz(1 - cos²)
    t2 = atan2(s2, c2);                                   % theta2 [rad], no quadrante correto
    t1 = atan2(y, x) - atan2(L2*s2, L1 + L2*c2);          % theta1 = ângulo do ponto - desvio causado pelo elo 2
    TH1(i,k) = rad2deg(t1);                               % guarda theta1 em graus
    TH2(i,k) = rad2deg(t2);                               % guarda theta2 em graus
  end
end

% ---------- Cinemática direta (posições para desenhar) ----------
X1 = L1*cosd(TH1);               Y1 = L1*sind(TH1);               % posição da junta 2 (fim do elo 1)
X2 = X1 + L2*cosd(TH1 + TH2);    Y2 = Y1 + L2*sind(TH1 + TH2);    % posição do punho (fim do elo 2)
% X2 e Y2 são os valores, produzidos pelos ângulos encontrados, que o robô efetivamente alcança; é a informação da última coluna da matriz de transformação homogênea final;
% É uma matirz 2xN, assim como eram TH1 e TH2, com uma linha para cada configuração do cotovelo

% ---------- Animação ----------
cor = {'r','b'};  nome = {'cima','baixo'};     % cores e nomes das duas configurações
for k = 1:N                                    % um quadro da animação por ponto
  subplot(2,6,[1,2,7,8]); cla; hold on         % área grande à esquerda; limpa só esse eixo
  plot(Caminho.x, Caminho.y, 'g*');            % desenha o caminho em verde
  for i = 1:2
    if isnan(TH1(i,k)), continue; end          % não desenha se não há solução neste ponto
    plot([0 X1(i,k) X2(i,k)], [0 Y1(i,k) Y2(i,k)], [cor{i} '-o'], 'LineWidth', 2);  % base -> junta 2 -> punho
  end
  axis equal; grid on; xlabel('x'); ylabel('y'); hold off   % mesma escala nos eixos, grade, rótulos

  for i = 1:2
    subplot(2,6,[3,4]+6*(i-1));                % gráfico da junta 1 (linha de cima: i=1; de baixo: i=2)
    plot(TH1(i,:), cor{i}); hold on            % curva de theta1 ao longo do caminho
    plot(k, TH1(i,k), 'ko'); hold off          % bolinha preta = ponto atual
    grid on; title(['Junta 1 / cotovelo ' nome{i}]); xlim([1 N]); ylim([-360 360])

    subplot(2,6,[5,6]+6*(i-1));                % gráfico da junta 2
    plot(TH2(i,:), cor{i}); hold on            % curva de theta2 ao longo do caminho
    plot(k, TH2(i,k), 'ko'); hold off          % bolinha preta = ponto atual
    grid on; title(['Junta 2 / cotovelo ' nome{i}]); xlim([1 N]); ylim([-360 360])
  end
  drawnow                                      % atualiza a tela a cada iteração (mais fluidez para a animação)
end
