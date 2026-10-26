% =====================
% Gradiente Descendente
% =====================
clear; clc; close all;

% Universo de discurso
ud = -3:0.05:1;
% Seja uma quadrática qualquer que se deseja encontrar o ponto de mínimo (não necessariamente uma função de erro).
% A quadrática desde exemplo é dada por: f(x) = x^2 + 2 * x + 1
f_ud = ud.^2 + 2 * ud + 1;

% Inicia-se em um ponto qualquer 'x' da quadrática:
x = (1 - -3) * rand - 3;

% Função objetivo a ser minimizada (a própria quadrática)
f = x^2 + 2 * x + 1;

% Bola que permite a condição de parada
ksi = 1e-5;

% Taxa de aprendizado
n = 0.01;

% Vertores auxiliares para plot
X = x; F = f;

while f > ksi
    % Calcule o gradiente de f(x) = x^2 + 2 * x + 1 da iteração atual
    % Grad f(x) = df(x)/dx = 2 * x + 2 (função de variável única)
    Grad = 2 * x + 2;

    % Atualize o ponto
    x = x - n * Grad;
    X = [X , x]; % armazena x atual para plot
    % Calcule a função objetivo a ser minimizada
    f = x^2 + 2 * x + 1;
    F = [F , f]; % armazena f atual para plot

    % Plot
    subplot(1,2,1)
    plot(ud , f_ud , 'b')
    hold on, plot(X , F , 'or'), plot(x , f , '*k'), hold off
    grid on, xlabel(['iteração [k = ' num2str(size(F,2)) ']'])
    ylabel('x_k'), title(['x_k = ' num2str(x)])
    subplot(1,2,2)
    plot(F , 'r' , 'linewidth' , 2); grid on;
    xlabel(['iteração [k = ' num2str(size(F,2)) ']']),
    axis([0 size(F,2) 0 4]), ylabel('f(x_k)'),
    title(['f_k = ' num2str(f)]), drawnow
end
