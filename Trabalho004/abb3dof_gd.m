% =====================================================================
%  CINEMÁTICA INVERSA POR GRADIENTE DESCENDENTE - ROBÔ ABB 3DoF
%  Trabalho 004 - Robótica Computacional (Octave)
%  Algoritmo: Gradiente Descendente para manipuladores (Aula 010, slide 24)
%  Cinemática direta: Denavit-Hartenberg (Alternativa A)
%  Objetivo: o end-effector (sem orientação) percorre o caminho do enunciado, partindo de Theta = zeros(3,1).
% =====================================================================
clear; clc; close all;               % limpa variáveis, tela e figuras

% ---------------------------------------------------------------------
% 1) PARÂMETROS DA SIMULAÇÃO (altere aqui para testar valores)
% ---------------------------------------------------------------------
L          = 3;      % comprimento de todos os elos: d1 = l2 = l3 = L
escala     = 3;      % fator de escala do caminho (alcançável até ~3.8)
eta        = 0.1;    % taxa de aprendizado η (tamanho do passo)
ksi        = 0.01;   % limiar ξ de convergência: o ponto é atingido quando EQM <= ξ
epsi       = 2;      % ε: se det(J'J) < ε, o Jacobiano é tratado como degenerado (slide 26 sugere η/5)
delta      = 0.1;    % valor δ somado à diagonal enquanto det < ε (Levenberg-Marquardt)
max_iter   = 5000;   % limite de iterações por ponto (evita laço infinito)
animar     = true;   % true = anima o robô percorrendo o caminho
passo_anim = 2;      % desenha 1 quadro a cada 'passo_anim' pontos
salvar     = true;   % true = salva as figuras em arquivos .png
estudos    = true;   % true = roda os estudos de η, ξ e escala
ndof       = 3;      % número de graus de liberdade do robô (M)

% ---------------------------------------------------------------------
% 2) FUNÇÕES AUXILIARES (precisam ser definidas antes de serem usadas)
% ---------------------------------------------------------------------

% Matriz de transformação de Denavit-Hartenberg: A = Tz(d)*Rz(th)*Tx(a)*Rx(alpha)
function A = dh(d, th, a, alpha)
  ct = cos(th); st = sin(th);              % cosseno e seno de theta
  ca = cos(alpha); sa = sin(alpha);        % cosseno e seno de alpha
  A = [ct, -st*ca,  st*sa, a*ct;           % linha 1 da matriz D-H genérica
       st,  ct*ca, -ct*sa, a*st;           % linha 2
       0,   sa,     ca,    d;              % linha 3 (d entra na translação em z)
       0,   0,      0,     1];             % linha 4 (homogênea)
end

% Cinemática direta do ABB: devolve F(:,:,i+1) = frame {i}, com i = 0..3
function F = fk_abb(th, L)
  tab = [L,  th(1),  0,  pi/2;             % junta 1: d=L, theta1, a=0,  alpha=+90 graus
         0,  th(2), -L,  0;                % junta 2: d=0, theta2, a=-L, alpha=0
         0,  th(3), -L,  0];               % junta 3: d=0, theta3, a=-L, alpha=0
  F = zeros(4,4,4);                        % guarda os 4 frames: {0}, {1}, {2}, {3}
  F(:,:,1) = eye(4);                       % frame {0}: base fixa na origem (F0 = I)
  for i = 1:3                              % para cada elo do robô
    F(:,:,i+1) = F(:,:,i) * dh(tab(i,1), tab(i,2), tab(i,3), tab(i,4)); % F_i = F_(i-1)*A_i
  end
end

% Jacobiano de posição ∇f(Q) (3 x M): derivada da posição do end-effector em relação a cada junta
function J = jacobiano(F)
  M = size(F,3) - 1;                       % número de juntas (frames menos a base)
  p = F(1:3,4,end);                        % posição do end-effector (última coluna de F_n)
  J = zeros(3,M);                          % aloca o Jacobiano
  for i = 1:M                              % uma coluna por junta
    z = F(1:3,3,i);                        % eixo de rotação da junta i = z do frame {i-1}
    o = F(1:3,4,i);                        % origem do frame {i-1}
    J(:,i) = cross(z, p - o);              % d(posição)/d(theta_i) para junta de rotação
  end
end

% Gradiente Descendente para manipuladores, percorrendo todo o caminho
function R = resolver_caminho(fk, ndof, Caminho, eta, ksi, delta, epsi, max_iter)
  Npts  = size(Caminho,2);                 % número de pontos do caminho
  Theta = zeros(ndof,1);                   % Q inicial: posição angular nula
  F     = fk(Theta);                       % frames na configuração inicial
  P     = F(1:3,4,end);                    % posição inicial do end-effector
  Traj  = P;                               % histórico da posição (para desenhar o rastro)
  R.Theta    = zeros(ndof,Npts);           % ângulos de junta em cada ponto do caminho
  R.iter     = zeros(1,Npts);              % iterações gastas em cada ponto
  R.EQM      = zeros(1,Npts);              % EQM final em cada ponto
  R.idx      = zeros(1,Npts);              % coluna de Traj ao fim de cada ponto
  R.hist     = cell(1,Npts);               % EQM a cada iteração, ponto a ponto
  R.completo = true;                       % vira false se algum ponto não convergir
  R.k_falha  = 0;                          % índice do primeiro ponto que falhou
  for k = 1:Npts                           % 2. Para cada ponto do caminho desejado
    G   = Caminho(:,k);                    % 2.1 ponto do caminho atual G_i
    EQM = 0.5 * (G - P)' * (G - P);        % 2.3 EQM = 1/2 [G - f(Q)]' [G - f(Q)]
    hist = EQM;                            % começa o histórico do EQM deste ponto
    it = 0;                                % contador de iterações deste ponto
    while EQM > ksi && it < max_iter && isfinite(EQM)   % 2.4 Enquanto EQM > ξ
      GradEQM = -(G - P);                  % 2.4.1 gradiente cartesiano: ∇EQM = -[G - f(Q)]
      J = jacobiano(F);                    % 2.4.2 Jacobiano de f(Q): ∇f(Q)
      A = J' * J;                          % matriz M x M dos mínimos quadrados
      cont = 0;                            % contador de proteção do laço de degeneração
      while det(A) < epsi && cont < 1000   % determinante pequeno = Jacobiano degenerado
        A = A + delta * eye(ndof);         % soma δ na diagonal (Levenberg-Marquardt)
        cont = cont + 1;                   % conta a correção
      end
      GradQ = A \ (J' * GradEQM);          % 2.4.3 ∇Q = (J'J)^-1 J' ∇EQM (slides 14 e 40)
      Theta = Theta - eta * GradQ;         % 2.4.4 Q(k+1) = Q(k) - η ∇Q
      F = fk(Theta);                       % 2.4.5 atualiza f(Q): novos frames
      P = F(1:3,4,end);                    % nova posição do end-effector
      EQM = 0.5 * (G - P)' * (G - P);      % 2.4.6 atualiza o EQM
      it = it + 1;                         % conta a iteração
      hist(end+1) = EQM;                   % guarda o EQM desta iteração
      Traj(:,end+1) = P;                   % guarda a posição (para desenhar o rastro)
    end
    R.Theta(:,k) = Theta;                  % guarda os ângulos encontrados
    R.iter(k)    = it;                     % guarda o número de iterações
    R.EQM(k)     = EQM;                    % guarda o EQM final
    R.idx(k)     = size(Traj,2);           % posição de Traj ao fim deste ponto
    R.hist{k}    = hist;                   % guarda o histórico do EQM
    if EQM > ksi                           % não convergiu (inalcançável ou η instável)
      R.completo = false;                  % marca a simulação como incompleta
      R.k_falha  = k;                      % registra onde falhou
      break;                               % interrompe o caminho
    end
  end
  R.Traj = Traj;                           % devolve o rastro completo do end-effector
end

% Desenha o robô: elos, juntas e eixos dos frames (x = vermelho, y = verde, z = azul)
function desenha_robo(F, comp_eixo)
  n = size(F,3);                           % número de frames
  pos = reshape(F(1:3,4,:), 3, n);         % posições das origens dos frames (3 x n)
  plot3(pos(1,:), pos(2,:), pos(3,:), '-', 'Color', [0.9 0.7 0], 'LineWidth', 4); % elos
  cores = 'rgb';                           % cores dos eixos x, y, z
  for i = 1:n                              % para cada frame
    o = F(1:3,4,i);                        % origem do frame {i-1}
    plot3(o(1), o(2), o(3), 'ko', 'MarkerSize', 9, 'LineWidth', 2); % marca a junta
    for c = 1:3                            % eixos x, y, z do frame
      ponta = o + comp_eixo * F(1:3,c,i);  % extremidade do eixo
      plot3([o(1) ponta(1)], [o(2) ponta(2)], [o(3) ponta(3)], cores(c), 'LineWidth', 1.5);
    end
    text(o(1), o(2), o(3) + 0.25*comp_eixo, sprintf('\\{%d\\}', i-1)); % rótulo do frame
  end
end

% Desenha a cena completa no ponto k do caminho
function cena_3d(fk, R, Caminho, k, comp_eixo, lim)
  clf; hold on;                            % limpa a figura e prepara para sobrepor
  plot3(Caminho(1,:), Caminho(2,:), Caminho(3,:), 'co', 'MarkerSize', 6); % caminho desejado
  plot3(R.Traj(1,1:R.idx(k)), R.Traj(2,1:R.idx(k)), R.Traj(3,1:R.idx(k)), 'm', 'LineWidth', 2); % rastro
  F0 = fk(zeros(size(R.Theta,1),1));       % configuração inicial (Theta = 0)
  pos0 = reshape(F0(1:3,4,:), 3, size(F0,3)); % origens dos frames iniciais
  plot3(pos0(1,:), pos0(2,:), pos0(3,:), ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 2); % robô inicial (cinza)
  desenha_robo(fk(R.Theta(:,k)), comp_eixo); % robô na configuração do ponto k
  P = R.Traj(:,R.idx(k));                  % posição atual do end-effector
  title(sprintf('End-effector: x = %.2f, y = %.2f, z = %.2f', P(1), P(2), P(3))); % título
  xlabel('x'); ylabel('y'); zlabel('z');   % rótulos dos eixos
  axis(lim); daspect([1 1 1]); grid on;    % limites fixos e mesma escala nos eixos
  view(135, 25);                           % ponto de vista 3D
  hold off; drawnow;                       % finaliza e atualiza a tela
end

% ---------------------------------------------------------------------
% 3) CINEMÁTICA DIRETA COMO FUNÇÃO DA CONFIGURAÇÃO
% ---------------------------------------------------------------------
fk = @(th) fk_abb(th, L);                  % atalho: fk(theta) devolve todos os frames

% ---------------------------------------------------------------------
% 4) VERIFICAÇÕES (confere o código contra a dedução analítica)
% ---------------------------------------------------------------------
th = [0.3; -0.7; 1.1];                     % configuração de teste qualquer
F = fk(th);                                % frames pelo código
c1 = cos(th(1)); s1 = sin(th(1));          % cos e sen de theta1
c2 = cos(th(2)); c23 = cos(th(2)+th(3));   % cos de theta2 e de theta2+theta3
s2 = sin(th(2)); s23 = sin(th(2)+th(3));   % sen de theta2 e de theta2+theta3
p_fechada = [-c1*(L*c2 + L*c23);           % posição pela fórmula fechada de F3 (Alternativa A)
             -s1*(L*c2 + L*c23);
              L - L*s2 - L*s23];
printf('Erro da cinemática direta vs fórmula fechada : %.2e\n', norm(F(1:3,4,end) - p_fechada));
Jn = zeros(3,ndof); h = 1e-6;              % Jacobiano numérico (diferenças finitas)
for i = 1:ndof                             % perturba uma junta de cada vez
  thh = th; thh(i) = thh(i) + h;           % theta perturbado
  Fh = fk(thh);                            % frames perturbados
  Jn(:,i) = (Fh(1:3,4,end) - F(1:3,4,end)) / h; % derivada parcial da posição
end
printf('Erro do Jacobiano vs diferenças finitas      : %.2e\n', norm(jacobiano(F) - Jn));
J0 = jacobiano(fk(zeros(ndof,1)));         % Jacobiano na configuração inicial (Theta = 0)
printf('det(J''J) em Theta = 0 (0 = Jacobiano degenerado) : %.2e\n\n', det(J0'*J0));

% ---------------------------------------------------------------------
% 5) CAMINHO DESEJADO (escalado)
% ---------------------------------------------------------------------
x = -2:0.05:2;                             % parâmetro do caminho: 81 pontos
Caminho = escala * [cos(2*pi*x/4);         % coordenada x do caminho
                    sin(2*pi*x/4);         % coordenada y do caminho
                    x.^2/2];               % coordenada z do caminho (parábola)
Npts = size(Caminho,2);                    % número de pontos

% ---------------------------------------------------------------------
% 6) SIMULAÇÃO PRINCIPAL
% ---------------------------------------------------------------------
R = resolver_caminho(fk, ndof, Caminho, eta, ksi, delta, epsi, max_iter); % roda o gradiente descendente
printf('=== ABB 3DoF | L=%g | escala=%g | eta=%g | ksi=%g | epsi=%g | delta=%g ===\n', L, escala, eta, ksi, epsi, delta);
printf('Caminho completo           : %d (1 = sim)\n', R.completo);
printf('Iterações totais           : %d\n', sum(R.iter));
printf('Iterações no 1º ponto      : %d (aproximação inicial, a partir de Theta = 0)\n', R.iter(1));
printf('Iterações médias (demais)  : %.1f\n', mean(R.iter(2:end)));
printf('Iterações máximas          : %d (ponto %d)\n', max(R.iter), find(R.iter == max(R.iter), 1));
printf('EQM final máximo           : %.4f (ksi = %g, distância máxima = %.3f)\n', max(R.EQM), ksi, sqrt(2*max(R.EQM)));
printf('Deslocamento angular total : %.2f rad\n', sum(sum(abs(diff(R.Theta,1,2)))));
printf('Ângulos finais (graus)     : %s\n', mat2str(rad2deg(R.Theta(:,end))', 4));
for i = 1:ndof                             % faixa percorrida por cada junta
  printf('  theta_%d: de %8.1f a %8.1f graus\n', i, rad2deg(min(R.Theta(i,:))), rad2deg(max(R.Theta(i,:)))); % mínimo e máximo
end

% ---------------------------------------------------------------------
% 7) GRÁFICOS
% ---------------------------------------------------------------------
if R.completo                              % só plota se o caminho foi concluído
  pts = Caminho;                           % reúne pontos para definir os limites dos eixos
  for k = 1:Npts                           % posições de todas as juntas ao longo do caminho
    Fk = fk(R.Theta(:,k));                 % frames no ponto k
    pts = [pts, reshape(Fk(1:3,4,:), 3, size(Fk,3))]; % acrescenta as origens dos frames
  end
  pts = [pts, zeros(3,1)];                 % inclui a base
  mn = min(pts,[],2); mx = max(pts,[],2);  % mínimo e máximo em cada eixo
  folga = 0.1 * max(mx - mn);              % margem de 10%
  lim = [mn(1)-folga mx(1)+folga mn(2)-folga mx(2)+folga mn(3)-folga mx(3)+folga]; % limites
  comp_eixo = 0.15 * max(mx - mn);         % comprimento dos eixos desenhados

  figure(1);                               % Figura 1: robô, caminho e rastro (3D)
  if animar                                % animação opcional
    for k = 1:passo_anim:Npts              % um quadro a cada 'passo_anim' pontos
      cena_3d(fk, R, Caminho, k, comp_eixo, lim); % desenha o ponto k
    end
  end
  cena_3d(fk, R, Caminho, Npts, comp_eixo, lim);  % quadro final (último ponto)
  if salvar, print('-dpng', '-r200', 'fig_abb_3d.png'); end % salva a figura

  figure(2);                               % Figura 2: ângulos das juntas
  plot(1:Npts, rad2deg(R.Theta)', 'LineWidth', 1.5); % ângulos em graus ao longo do caminho
  grid on; xlabel('Ponto do caminho (k)'); ylabel('Angulo (graus)'); % rótulos
  legend('theta_1', 'theta_2', 'theta_3', 'Location', 'best');       % legenda
  title('ABB 3DoF - angulos das juntas');  % título
  if salvar, print('-dpng', '-r200', 'fig_abb_angulos.png'); end

  figure(3);                               % Figura 3: iterações gastas em cada ponto
  bar(1:Npts, R.iter);                     % barras: iterações por ponto do caminho
  grid on; xlabel('Ponto do caminho (k)'); ylabel('Iteracoes'); % rótulos
  title('ABB 3DoF - iteracoes por ponto'); % título
  if salvar, print('-dpng', '-r200', 'fig_abb_iteracoes.png'); end
end

% ---------------------------------------------------------------------
% 8) ESTUDOS DE PARÂMETROS (η, ξ e escala do caminho)
% ---------------------------------------------------------------------
if estudos
  % 8.1) Tabela η x ξ: iterações totais ('X' = não concluiu o caminho)
  etas = [0.05 0.1 0.5 1.0 1.5 2.1];       % taxas de aprendizado testadas
  ksis = [0.1 0.01 0.001 0.0001];          % limiares de convergência testados
  curvas = cell(1,numel(etas));            % guarda o EQM do 1º ponto (ξ = 0.01)
  printf('\n--- Estudo 1: iterações totais (escala=%g, epsi=%g, delta=%g) ---\n', escala, epsi, delta);
  printf('%8s', 'eta\ksi'); printf('%10g', ksis); printf('\n');
  for a = 1:numel(etas)                    % para cada η
    printf('%8g', etas(a));                % imprime o η da linha
    for b = 1:numel(ksis)                  % para cada ξ
      Re = resolver_caminho(fk, ndof, Caminho, etas(a), ksis(b), delta, epsi, 2000); % roda a simulação
      if Re.completo, printf('%10d', sum(Re.iter)); else, printf('%10s', 'X'); end % total ou X
      if ksis(b) == 0.01, curvas{a} = Re.hist{1}; end % guarda histórico do 1º ponto
    end
    printf('\n');
  end

  % 8.2) Escala do caminho (alcançabilidade)
  escalas = [1 2 3 3.5 3.8 4];             % escalas testadas
  printf('\n--- Estudo 2: escala do caminho (eta=%g, ksi=%g) ---\n', eta, ksi);
  for a = 1:numel(escalas)                 % para cada escala
    Cs = escalas(a) * [cos(2*pi*x/4); sin(2*pi*x/4); x.^2/2]; % caminho com a nova escala
    Re = resolver_caminho(fk, ndof, Cs, eta, ksi, delta, epsi, 2000); % roda a simulação
    if Re.completo, printf('escala = %-4g iterações totais = %d (1º ponto: %d)\n', escalas(a), sum(Re.iter), Re.iter(1));
    else, printf('escala = %-4g NÃO ALCANÇÁVEL (falhou no ponto %d; EQM %.3f)\n', escalas(a), Re.k_falha, Re.EQM(Re.k_falha)); end
  end

  figure(4);                               % Figura 4: convergência do 1º ponto para vários η
  hold on;                                 % sobrepõe as curvas
  for a = 1:numel(etas)                    % uma curva por η
    if ~isempty(curvas{a}), semilogy(0:numel(curvas{a})-1, curvas{a}, 'LineWidth', 1.5); end
  end
  semilogy([0 200], [ksi ksi], 'k--');     % linha do limiar ksi
  hold off; grid on;                       % finaliza
  xlabel('Iteracao'); ylabel('EQM');       % rótulos
  title('ABB 3DoF - EQM no 1o ponto (ksi = 0.01)'); % título
  axis([0 200 1e-6 1e2]);                  % limita os eixos
  legend(arrayfun(@(v) sprintf('eta = %g', v), etas, 'UniformOutput', false), 'Location', 'northeast');
  if salvar, print('-dpng', '-r200', 'fig_abb_eta.png'); end
end
