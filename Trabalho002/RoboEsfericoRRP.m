clear; close all; clc;

## Pacote simbólico
pkg load symbolic

% =========================================================================
% ROBO ESFÉRICO (RRP) - Shoulder (giro) + Shoulder (elevação) + extensão
% Cinemática direta via Denavit-Hartenberg (100% simbólico)
% Ordem de multiplicação definida pela disciplina:  Tz . Rz . Tx . Rx
% =========================================================================

% --------- Simbólico base ---------
syms th dx dy dz L1 L2 real
syms th1 th2 d3 real

one = sym(1);
zero = sym(0);
deg2rad_sym = @(d) sym(d) * sym(pi) / sym(180);

% Matriz de translação genérica (dx, dy, dz)
T = [ one  zero zero dx;
      zero one  zero dy;
      zero zero one  dz;
      zero zero zero one ];

% Matriz de rotação em X
Rx = [ one  zero    zero     zero;
       zero cos(th) -sin(th) zero;
       zero sin(th) cos(th)  zero;
       zero zero    zero     one ];

% Matriz de rotação em Z
Rz = [ cos(th) -sin(th) zero zero;
       sin(th) cos(th)  zero zero;
       zero    zero     one  zero;
       zero    zero     zero one ];

F0 = sym(eye(4));

% =========================================================================
% TABELA DE PARÂMETROS DH
%   i |  theta_i  |  d_i  |  a_i  | alpha_i
%   1 |   th1*    |  L1   |  0    | -90°
%   2 |  -th2*    |  L2   |  0    | -90°
%   3 |   0       |  d3*  |  0    |  0°
% =========================================================================

pi2 = sym(pi)/2;

% --------- Junta 1: A1 = Tz(L1) . Rz(th1) . Tx(0) . Rx(-90°) ---------
Tz1 = subs(T, [dx dy dz], [zero zero L1]);
Rz1 = subs(Rz, th, th1);
Tx1 = subs(T, [dx dy dz], [zero zero zero]);
Rx1 = subs(Rx, th, -pi2);
A1  = Tz1 * Rz1 * Tx1 * Rx1;
F1  = F0 * A1;

% --------- Junta 2: A2 = Tz(L2) . Rz(-th2) . Tx(0) . Rx(-90°) ---------
Tz2 = subs(T, [dx dy dz], [zero zero L2]);
Rz2 = subs(Rz, th, -th2);
Tx2 = subs(T, [dx dy dz], [zero zero zero]);
Rx2 = subs(Rx, th, -pi2);
A2  = Tz2 * Rz2 * Tx2 * Rx2;
F2  = F1 * A2;

% --------- Junta 3: A3 = Tz(d3) . Rz(0) . Tx(0) . Rx(0°) ---------
Tz3 = subs(T, [dx dy dz], [zero zero d3]);
Rz3 = subs(Rz, th, zero);
Tx3 = subs(T, [dx dy dz], [zero zero zero]);
Rx3 = subs(Rx, th, zero);
A3  = Tz3 * Rz3 * Tx3 * Rx3;
F3  = F2 * A3;

A1 = simplify(A1);
A2 = simplify(A2);
A3 = simplify(A3);
F1 = simplify(F1);
F2 = simplify(F2);
F3 = simplify(F3);

disp('--- A1 ---'); disp(A1);
disp('--- A2 ---'); disp(A2);
disp('--- A3 ---'); disp(A3);
disp('--- F3 = A1*A2*A3 (Cinemática Direta - Matriz Homogênea Final) ---'); disp(F3);

% =========================================================================
% POSE NUMÉRICA DE EXEMPLO (apenas para gerar o plot 3D do robô)
% A derivação acima permanece 100% simbólica; esta seção só substitui valores numéricos em F1, F2, F3 para desenhar UMA configuração específica.
% =========================================================================
l1s = sym('0.3');   % comprimento do elo 1 (base -> ombro)
l2s = sym('0.25');  % comprimento do elo 2 (ombro -> junta prismática)
d3s = sym('0.2');   % extensão da junta prismática

t1s = deg2rad_sym(30);   % pose escolhida para o plot
t2s = deg2rad_sym(40);

F0n = eye(4);
F1n = double(vpa(subs(F1, [L1 th1], [l1s t1s]), 12));
F2n = double(vpa(subs(F2, [L1 L2 th1 th2], [l1s l2s t1s t2s]), 12));
F3n = double(vpa(subs(F3, [L1 L2 th1 th2 d3], [l1s l2s t1s t2s d3s]), 12));

% ==== Plot ====
figure(1); clf; axis equal; grid on; view(3);
xlabel('x'); ylabel('y'); zlabel('z');
title('Robô Esférico (RRP) - Configuração exemplo');
hold on;
esc = 0.1; mark = 5;

% ---------- Frame {0} ----------
plot3(F0n(1,4), F0n(2,4), F0n(3,4), 'om', 'linewidth', 2, 'markersize', mark);
plot3([F0n(1,4) F0n(1,4)+esc*F0n(1,1)], [F0n(2,4) F0n(2,4)+esc*F0n(2,1)], [F0n(3,4) F0n(3,4)+esc*F0n(3,1)], 'b', 'linewidth', 2);
text( F0n(1,4)+esc*F0n(1,1), F0n(2,4)+esc*F0n(2,1), F0n(3,4)+esc*F0n(3,1), 'x_{\{0\}}');
plot3([F0n(1,4) F0n(1,4)+esc*F0n(1,2)], [F0n(2,4) F0n(2,4)+esc*F0n(2,2)], [F0n(3,4) F0n(3,4)+esc*F0n(3,2)], 'g', 'linewidth', 2);
text( F0n(1,4)+esc*F0n(1,2), F0n(2,4)+esc*F0n(2,2), F0n(3,4)+esc*F0n(3,2), 'y_{\{0\}}');
plot3([F0n(1,4) F0n(1,4)+esc*F0n(1,3)], [F0n(2,4) F0n(2,4)+esc*F0n(2,3)], [F0n(3,4) F0n(3,4)+esc*F0n(3,3)], 'r', 'linewidth', 2);
text( F0n(1,4)+esc*F0n(1,3), F0n(2,4)+esc*F0n(2,3), F0n(3,4)+esc*F0n(3,3), 'z_{\{0\}}');
plot3([F0n(1,4) F1n(1,4)], [F0n(2,4) F1n(2,4)], [F0n(3,4) F1n(3,4)], 'k');

% ---------- Frame {1} ----------
plot3(F1n(1,4), F1n(2,4), F1n(3,4), 'om','linewidth',2,'markersize',mark);
text(F1n(1,4), F1n(2,4), F1n(3,4)-0.2, '\{1\}')
plot3([F1n(1,4) F1n(1,4)+esc*F1n(1,1)], [F1n(2,4) F1n(2,4)+esc*F1n(2,1)], [F1n(3,4) F1n(3,4)+esc*F1n(3,1)], 'b','linewidth',2)
text(F1n(1,4)+esc*F1n(1,1), F1n(2,4)+esc*F1n(2,1), F1n(3,4)+esc*F1n(3,1), 'x_{\{1\}}')
plot3([F1n(1,4) F1n(1,4)+esc*F1n(1,2)], [F1n(2,4) F1n(2,4)+esc*F1n(2,2)], [F1n(3,4) F1n(3,4)+esc*F1n(3,2)], 'g','linewidth',2)
text(F1n(1,4)+esc*F1n(1,2), F1n(2,4)+esc*F1n(2,2), F1n(3,4)+esc*F1n(3,2), 'y_{\{1\}}')
plot3([F1n(1,4) F1n(1,4)+esc*F1n(1,3)], [F1n(2,4) F1n(2,4)+esc*F1n(2,3)], [F1n(3,4) F1n(3,4)+esc*F1n(3,3)], 'r','linewidth',2)
text(F1n(1,4)+esc*F1n(1,3), F1n(2,4)+esc*F1n(2,3), F1n(3,4)+esc*F1n(3,3), 'z_{\{1\}}')
plot3([F1n(1,4) F2n(1,4)], [F1n(2,4) F2n(2,4)], [F1n(3,4) F2n(3,4)], 'k')

% ---------- Frame {2} ----------
plot3(F2n(1,4), F2n(2,4), F2n(3,4), 'om','linewidth',2,'markersize',mark);
text(F2n(1,4), F2n(2,4), F2n(3,4)-0.2, '\{2\}')
plot3([F2n(1,4) F2n(1,4)+esc*F2n(1,1)], [F2n(2,4) F2n(2,4)+esc*F2n(2,1)], [F2n(3,4) F2n(3,4)+esc*F2n(3,1)], 'b','linewidth',2)
text(F2n(1,4)+esc*F2n(1,1), F2n(2,4)+esc*F2n(2,1), F2n(3,4)+esc*F2n(3,1), 'x_{\{2\}}')
plot3([F2n(1,4) F2n(1,4)+esc*F2n(1,2)], [F2n(2,4) F2n(2,4)+esc*F2n(2,2)], [F2n(3,4) F2n(3,4)+esc*F2n(3,2)], 'g','linewidth',2)
text(F2n(1,4)+esc*F2n(1,2), F2n(2,4)+esc*F2n(2,2), F2n(3,4)+esc*F2n(3,2), 'y_{\{2\}}')
plot3([F2n(1,4) F2n(1,4)+esc*F2n(1,3)], [F2n(2,4) F2n(2,4)+esc*F2n(2,3)], [F2n(3,4) F2n(3,4)+esc*F2n(3,3)], 'r','linewidth',2)
text(F2n(1,4)+esc*F2n(1,3), F2n(2,4)+esc*F2n(2,3), F2n(3,4)+esc*F2n(3,3), 'z_{\{2\}}')
plot3([F2n(1,4) F3n(1,4)], [F2n(2,4) F3n(2,4)], [F2n(3,4) F3n(3,4)], 'k')

% ---------- Frame {3} ----------
plot3(F3n(1,4), F3n(2,4), F3n(3,4), 'om','linewidth',2,'markersize',mark);
text(F3n(1,4), F3n(2,4), F3n(3,4)-0.2, '\{3\}')
plot3([F3n(1,4) F3n(1,4)+esc*F3n(1,1)], [F3n(2,4) F3n(2,4)+esc*F3n(2,1)], [F3n(3,4) F3n(3,4)+esc*F3n(3,1)], 'b','linewidth',2)
text(F3n(1,4)+esc*F3n(1,1), F3n(2,4)+esc*F3n(2,1), F3n(3,4)+esc*F3n(3,1), 'x_{\{3\}}')
plot3([F3n(1,4) F3n(1,4)+esc*F3n(1,2)], [F3n(2,4) F3n(2,4)+esc*F3n(2,2)], [F3n(3,4) F3n(3,4)+esc*F3n(3,2)], 'g','linewidth',2)
text(F3n(1,4)+esc*F3n(1,2), F3n(2,4)+esc*F3n(2,2), F3n(3,4)+esc*F3n(3,2), 'y_{\{3\}}')
plot3([F3n(1,4) F3n(1,4)+esc*F3n(1,3)], [F3n(2,4) F3n(2,4)+esc*F3n(2,3)], [F3n(3,4) F3n(3,4)+esc*F3n(3,3)], 'r','linewidth',2)
text(F3n(1,4)+esc*F3n(1,3), F3n(2,4)+esc*F3n(2,3), F3n(3,4)+esc*F3n(3,3), 'z_{\{3\}}')
