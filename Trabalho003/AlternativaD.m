% ======================================================
% Robô planar 2GL - Alternativa D: curva de coração
% ======================================================
% Fonte da curva: Weisstein, Eric W. "Heart Curve." MathWorld--A Wolfram Web
% Resource. http://mathworld.wolfram.com/HeartCurve.html
% Curva usada (primeira equação paramétrica da página), com t de 0 a 2*pi:
%     x = 16 * sen(t)^3
%     y = 13*cos(t) - 5*cos(2t) - 2*cos(3t) - cos(4t)
% ======================================================
clc; clear; close all;          % limpa tela, variáveis e figuras

L1 = 1; L2 = 1;                 % comprimentos dos elos 1 e 2

% ---------- Caminho: curva de coração adaptada ao robô ----------
% A curva original tem cerca de 32 de largura por 29 de altura, mas o robô
% só alcança pontos a no máximo L1 + L2 = 2 da base. Por isso é preciso reduzi-la.
esc = 0.1;                      % fator de escala: reduz a curva a 10% do tamanho original
dy  = 0.25;                     % deslocamento vertical: centraliza o coração em torno da base (0,0)

t = linspace(0, 2*pi, 100);     % 100 valores do parâmetro t, de 0 a 2*pi (curva fechada)
Caminho.x = esc * 16 * sin(t).^3;                                              % x do coração (.^ opera elemento a elemento)
Caminho.y = esc * (13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t)) + dy;       % y do coração, escalado e deslocado
% Com esses valores, todos os pontos ficam entre 0,75 e 1,82 de distância da base,
% ou seja, dentro do workspace (de 0 a 2).

% ---------- Matrizes de resultado ----------
N = length(Caminho.x);          % número de pontos do caminho
TH1 = NaN(2,N);                 % ângulos da junta 1 [graus]: linha 1 = cotovelo cima, linha 2 = baixo
TH2 = NaN(2,N);                 % ângulos da junta 2 [graus]; NaN = ponto sem solução

% ---------- Cinemática inversa ----------
for k = 1:N                     % percorre cada ponto do caminho
  x = Caminho.x(k);  y = Caminho.y(k);                    % ponto desejado para o punho
  c2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);            % cos(theta2)
  if abs(c2) > 1, continue; end                           % -1 <= cos(theta2) <= 1? Se não, fora do workspace: fica NaN

  sinais = [-1, 1];                                       % -1 = cotovelo para cima, +1 = para baixo
  for i = 1:2                                             % calcula as duas configurações
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

% Verificação: o punho deve coincidir com o caminho (erro da ordem de 1e-15)
disp(max(abs([X2(1,:) - Caminho.x, Y2(1,:) - Caminho.y])))

% ---------- Animação ----------
cor = {'r','b'};  nome = {'cima','baixo'};     % cores e nomes das duas configurações
for k = 1:N                                    % um quadro da animação por ponto
  subplot(2,6,[1,2,7,8]); cla; hold on         % área grande à esquerda; limpa só esse eixo
  plot(Caminho.x, Caminho.y, 'g*');            % desenha o coração em verde
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
  drawnow                                      % atualiza a tela a cada iteração
end
