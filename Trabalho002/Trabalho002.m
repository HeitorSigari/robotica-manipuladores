clear; close all; clc;

## Pacote simbólico
pkg load symbolic

% ----- Simbólico base -----
syms th a d real
syms th1 th2 th3 th4 th5 th6 th7 real

one = sym(1);
zero = sym(0);
deg2rad_sym = @(d) sym(d) * sym(pi) / sym(180);

% ----- Matrizes Simbólicas -----
% Matriz de translação em X
Tx = [ one  zero zero a;
      zero one  zero zero;
      zero zero one  zero;
      zero zero zero one ];

% Matriz de rotação em X
Rx = [ one  zero    zero     zero;
       zero cos(th) -sin(th) zero;
       zero sin(th) cos(th)  zero;
       zero zero    zero     one ];

%Matriz de translação em Z
Tz = [ one  zero zero zero;
      zero one  zero zero;
      zero zero one  d;
      zero zero zero one ];

% Matriz de rotação em Z
Rz = [ cos(th) -sin(th) zero zero;
       sin(th) cos(th)  zero zero;
       zero    zero     one  zero;
       zero    zero     zero one ];
