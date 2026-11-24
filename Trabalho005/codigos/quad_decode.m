% quad_decode.m - Decodificador de encoder em quadratura (funcao)
function [cnt, hist] = quad_decode(A, B)
  % A, B : vetores de amostras (0 ou 1) dos canais A e B
  % cnt  : contagem liquida (com sinal) ao final da sequencia
  % hist : contagem acumulada apos cada amostra
  mapa = [4 3; 1 2];                              % mapa(A+1,B+1) -> estado: S4=(0,0) S3=(0,1) S1=(1,0) S2=(1,1)
  tab = zeros(4,4);                               % tab(anterior,novo) = incremento (+1, -1 ou 0)
  for i = 1:4                                     % monta a tabela de transicao
    prox = mod(i,4) + 1;                          % proximo estado no sentido positivo (S4 volta a S1)
    tab(i,prox) = +1;                             % Si -> Si+1 : sentido positivo
    tab(prox,i) = -1;                             % Si+1 -> Si : sentido negativo
  end
  N = numel(A);                                   % numero de amostras
  hist = zeros(1,N);                              % acumulador (comeca em zero)
  s = mapa(A(1)+1, B(1)+1);                       % estado inicial a partir da primeira amostra
  cnt = 0;                                        % contagem acumulada
  for k = 2:N                                     % percorre as amostras seguintes
    s2 = mapa(A(k)+1, B(k)+1);                    % estado atual
    cnt = cnt + tab(s,s2);                        % soma +1, -1 ou 0 conforme a transicao
    hist(k) = cnt;                                % guarda a contagem acumulada
    s = s2;                                       % atual passa a ser o anterior
  end
end
