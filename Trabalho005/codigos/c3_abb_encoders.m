% c3_abb_encoders.m - Alternativa C.3: encoders nas juntas do ABB 3DoF
clc; clear; close all;                            % limpa tela, variaveis e figuras

L = [3 3 3];                                      % comprimentos dos elos L1, L2, L3
q = [35 296 250];                                 % configuracao [th1 th2 th3] em graus
[p0, F] = fk_abb(q, L);                           % posicao e matriz homogenea na configuracao dada
disp('Matriz de transformacao homogenea F:'); disp(F);
printf('Posicao do efetuador: x = %.4f | y = %.4f | z = %.4f\n', p0(1), p0(2), p0(3));

ticks = 2048;                                     % ticks por revolucao do encoder
res_motor = 360/(4*ticks);                        % resolucao no eixo do motor, em quadratura [graus]
printf('Resolucao no eixo do motor: %.6f graus\n', res_motor);

reducoes = [100 1];                               % reducao 100:1 (item b) e 1:1 (item c)
for i = 1:numel(reducoes)                         % repete o calculo para cada reducao
  red = reducoes(i);                              % reducao atual
  res_junta = res_motor/red;                      % resolucao angular da junta [graus]
  printf('\nReducao %d:1 -> resolucao da junta = %.6f graus (%.3e rad)\n', red, res_junta, deg2rad(res_junta));
  for j = 1:3                                     % desloca uma junta por vez
    dq = q;                                       % copia a configuracao original
    dq(j) = dq(j) + res_junta;                    % soma UMA contagem do encoder na junta j
    dp = fk_abb(dq, L) - p0;                      % deslocamento do efetuador [dx; dy; dz]
    printf('  junta %d: dx = %+.3e | dy = %+.3e | dz = %+.3e | norma = %.3e\n', ...
           j, dp(1), dp(2), dp(3), norm(dp));
  end
end
