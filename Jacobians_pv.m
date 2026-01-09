clear
clc
syms ST H11 H12 H13 H21 H22 H23 H31 H32 H33 H41 H42 H43 H51 H52 H53 H61 H62 H63

F = [ eye(3) eye(3)*ST; zeros(3,3) eye(3)];

H = [ H11 H12 H13 0 0 0;
      H21 H22 H23 0 0 0;
      H31 H32 H33 0 0 0;
      H41 H42 H43 0 0 0;
      H51 H52 H53 0 0 0;
      H61 H62 H63 0 0 0];

% Osservability 

O = [ H; H*F];
rO=rank(O);

fprintf('Osservability matrix rank is: %d\n', rO)

