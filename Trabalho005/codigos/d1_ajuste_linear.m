% d1_ajuste_linear.m - Alternativa D.1: ajuste linear c(T) = c0 + a*T pelo metodo dos minimos quadrados
% Formula fechada: Theta = (X'*X)^(-1) * X' * y
clc; clear; close all;                            % limpa tela, variaveis e figuras
if ~exist('figuras','dir'), mkdir('figuras'); end % cria a pasta de figuras se nao existir

T = (-10:5:30)';                                  % temperaturas medidas [graus C], vetor coluna 9x1
y = [325.4 328.5 331.5 334.5 337.5 340.5 343.4 346.3 349.2]';   % velocidades do som medidas [m/s], vetor coluna 9x1

N = numel(T);                                     % numero de pontos (N = 9)
X = [T, ones(N,1)];                               % matriz X (N x 2): 1a coluna = T (multiplica a), 2a coluna = 1 (multiplica c0)
XtX = X'*X;                                       % matriz X'X (2 x 2)
Xty = X'*y;                                       % vetor X'y (2 x 1)
Theta = inv(XtX)*Xty;                             % formula
a  = Theta(1);                                    % coeficiente angular [m/s por grau C]
c0 = Theta(2);                                    % coeficiente linear (velocidade a 0 grau C) [m/s]

z = X*Theta;                                      % valores da reta ajustada em cada temperatura [m/s]
res = y - z;                                      % residuos = dado medido - reta ajustada [m/s]
[rmax, i] = max(abs(res));                        % maior residuo em modulo e sua posicao
disp('X''X ='); disp(XtX);                        % mostra X'X (= [soma T^2, soma T; soma T, N])
disp('X''y ='); disp(Xty);                        % mostra X'y (= [soma T*y; soma y])
printf('c0 = %.4f m/s | a = %.4f m/s por grau C\n', c0, a);
printf('Maior residuo = %.4f m/s (em T = %d graus C; residuo com sinal = %+.4f)\n', rmax, T(i), res(i));
disp(['Residuos: ' num2str(res', '%.3f ')]);     % lista todos os residuos
p = polyfit(T, y, 1);                             % conferencia: ajuste interno do Octave (deve dar o mesmo resultado)
printf('Conferencia com polyfit: diferenca em a = %.2e | diferenca em c0 = %.2e\n', abs(p(1)-a), abs(p(2)-c0));

figure('Position', [100 100 700 450]);            % abre a figura
plot(T, y, 'bo', 'MarkerFaceColor', 'b'); hold on;% dados medidos (circulos azuis)
Tf = linspace(-10, 30, 100);                      % temperaturas para desenhar a reta
plot(Tf, c0 + a*Tf, 'r-', 'LineWidth', 1.5);      % reta ajustada (vermelha): c0 + a*T
grid on;                                          % grade
xlabel('Temperatura T (graus C)');                % eixo x identificado
ylabel('Velocidade do som c (m/s)');              % eixo y identificado
title('Velocidade do som no ar: dados e ajuste linear');   % titulo
legend('Dados medidos', sprintf('c(T) = %.2f + %.4f T', c0, a), 'Location', 'northwest');   % legenda
print('-dpng', '-r150', 'figuras/sonar_ajuste.png');         % salva a figura
