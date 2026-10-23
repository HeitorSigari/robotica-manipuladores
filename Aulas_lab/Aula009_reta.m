clear; clc; close all;

% Valores dos parâmetros de reta; são sorteados a cada execução de código pelo comando randn
th1 = randn(); % coeficiente 1
th2 = randn(); % coeficiente 2

% Valores em x
x = (-10:0.1:10)'; % vetor linha de -10 a 10 com passo 0,1 (201 pontos); o apóstrofo no final transpõe o vetor.

% Valores do sistema (y)
y = th1 * x + th2 + randn(size(x));
% randn gera um vetor 201×1 de ruído gaussiano, um valor independente para cada ponto.
% assim, temos uma reta verdadeira contaminada por ruído de variância 1.

% não se conhece os parâmetros para obter a reta que minimiza o erro, quer-se determiná-los
% é possível determiná-los por meio da fórmula fechada obtida pelo método dos mínimos quadrados (MMQ) de Gauss, como abaixo:
X = [x , ones(size(x))]; % Monta a matriz de regressão (201×2): a primeira coluna é x e a segunda é só de 1s (ones(size(x))).
TH = inv(X' * X) * X' * y;
% o resultado TH é um vetor 2×1 com as estimativas: TH(1) ≈ th1 e TH(2) ≈ th2.
% assim, X * [th1; th2] produz th1·x + th2 = z, ou seja, o modelo linear na forma matricial y ≈ Xθ (X.TH). A coluna de 1s existe para representar o termo constante (intercepto).

% Obtendo os valores da melhor reta ajustada (já com os parâmetros theta1 e theta2 obtidos).
z = TH(1) * x + TH(2);

% Plotagem
figure;
plot(x, y, '.k', x, z, 'r', 'linewidth', 2, 'markersize', 10);
xlabel('x');
ylabel('y');
title('Mínimos Quadrados');
grid on;
axis equal;
legend('Sistema com ruído', 'Ajuste com MMQ');

% Exibição dos resultados
disp(['Valor calculado: th1 = ' num2str(TH(1)) ', th2 = ' num2str(TH(2))]);

