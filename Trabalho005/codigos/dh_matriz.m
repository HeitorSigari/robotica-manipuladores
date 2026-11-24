% dh_matriz.m - Matriz de transformacao homogenea de Denavit-Hartenberg (funcao)
function T = dh_matriz(theta, d, a, alpha)
  % theta, alpha em graus ; d, a em unidades de comprimento
  ct = cosd(theta); st = sind(theta);             % cosseno e seno de theta
  ca = cosd(alpha); sa = sind(alpha);             % cosseno e seno de alpha
  T = [ct, -st*ca,  st*sa, a*ct;                  % linha 1: rotacao e translacao em x
       st,  ct*ca, -ct*sa, a*st;                  % linha 2: rotacao e translacao em y
       0,      sa,     ca,   d;                   % linha 3: rotacao e translacao em z
       0,       0,      0,   1];                  % linha 4: parte homogenea
end
