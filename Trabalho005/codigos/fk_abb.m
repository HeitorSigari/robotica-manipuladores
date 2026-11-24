% fk_abb.m - Cinematica direta do robo articulado RRR (ABB 3DoF) pela tabela D-H (funcao)
function [p, F] = fk_abb(q, L)
  % q = [th1 th2 th3] em graus ; L = [L1 L2 L3]
  % Tabela D-H: (d, theta, a, alpha) = (L1, th1, 0, -90) ; (0, th2, L2, 0) ; (0, th3, L3, 0)
  A1 = dh_matriz(q(1), L(1), 0,    -90);          % elo 1: base giratoria
  A2 = dh_matriz(q(2), 0,    L(2),   0);          % elo 2: ombro
  A3 = dh_matriz(q(3), 0,    L(3),   0);          % elo 3: cotovelo
  F = A1*A2*A3;                                   % matriz de transformacao homogenea final
  p = F(1:3,4);                                   % posicao do efetuador [x; y; z]
end
