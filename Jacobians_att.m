clear
clc
syms pw px py pz g mx my mz omega_b_x omega_b_y omega_b_z omega_bias_x omega_bias_y omega_bias_z ST 

omega_bias = [omega_bias_x omega_bias_y omega_bias_z]';
omega_b = [omega_b_x omega_b_y omega_b_z]';
quater = [pw px py pz]';

phi_vec = (omega_b - omega_bias)*ST;
phi = norm(phi_vec);
u= phi_vec/phi;
qw = cos(phi/2);
qx = u(1) * sin(phi/2);
qy = u(2) * sin(phi/2);
qz = u(3) * sin(phi/2);

f1 = pw*qw - px*qx - py*qy - pz*qz;
f2 = pw*qx + px*qw + py*qz - pz*qy;
f3 = pw*qy - px*qz + py*qw + pz*qx;
f4 = pw*qz + px*qy - py*qx + pz*qw;
f5 = omega_bias_x;
f6 = omega_bias_y;
f7 = omega_bias_z;

omega_bias = omega_bias';
omega_b = omega_b';
quater = quater';

Fq1 = jacobian(f1,quater);
Fq2 = jacobian(f2,quater);
Fq3 = jacobian(f3,quater);
Fq4 = jacobian(f4,quater);
Fq5 = jacobian(f5,quater);
Fq6 = jacobian(f6,quater);
Fq7 = jacobian(f7,quater);

Fo1 = jacobian(f1,omega_bias);
Fo2 = jacobian(f2,omega_bias);
Fo3 = jacobian(f3,omega_bias);
Fo4 = jacobian(f4,omega_bias);
Fo5 = jacobian(f5,omega_bias);
Fo6 = jacobian(f6,omega_bias);
Fo7 = jacobian(f7,omega_bias);

F = [Fq1 Fo1;
     Fq2 Fo2;
     Fq3 Fo3;
     Fq4 Fo4;
     Fq5 Fo5;
     Fq6 Fo6;
     Fq7 Fo7];

D1 = jacobian(f1,omega_b);
D2 = jacobian(f2,omega_b);
D3 = jacobian(f3,omega_b);
D4 = jacobian(f4,omega_b);

D = [D1; D2; D3; D4; zeros(3,3)];

matlabFunction(F,'file','F_attitude')
matlabFunction(D,'file','D_attitude')

R_ob = [qw^2+qx^2-qy^2-qz^2      2*(qx*qy-qw*qz)       2*(qx*qz+qw*qy);
           2*(qx*qy+qw*qz)  qw^2-qx^2+qy^2-qz^2       2*(qy*qz-qw*qx);
           2*(qx*qz-qw*qy)      2*(qy*qz+qw*qx)   qw^2-qx^2-qy^2+qz^2];

R_q =  [pw^2+px^2-py^2-pz^2      2*(px*py-pw*pz)       2*(px*pz+pw*py);
            2*(px*py+pw*pz)  pw^2-px^2+py^2-pz^2       2*(py*pz-pw*px);
            2*(px*pz-pw*py)      2*(py*pz+pw*px)   pw^2-px^2-py^2+pz^2];

gn=[0 0 g]';
mn=[mx my mz]';

h1_q=-R_q*gn;
h2_q=R_q*mn;

h1_ob=R_ob*gn;
h2_ob=R_ob*mn;

Hq1=jacobian(h1_q(1),quater);
Hq2=jacobian(h1_q(2),quater);
Hq3=jacobian(h1_q(3),quater);
Hq4=jacobian(h2_q(1),quater);
Hq5=jacobian(h2_q(2),quater);
Hq6=jacobian(h2_q(3),quater);

Hb1=jacobian(h1_ob(1),omega_bias);
Hb2=jacobian(h1_ob(2),omega_bias);
Hb3=jacobian(h1_ob(3),omega_bias);
Hb4=jacobian(h2_ob(1),omega_bias);
Hb5=jacobian(h2_ob(2),omega_bias);
Hb6=jacobian(h2_ob(3),omega_bias);

Hq = [Hq1; Hq2; Hq3; Hq4; Hq5; Hq6];
Hb = [Hb1; Hb2; Hb3; Hb4; Hb5; Hb6];

H = [ Hq Hb ];

matlabFunction(H,'file','H_attitude')

%% Osservability
O = [H; H*F];
rO=rank(O);

fprintf('Osservability matrix rank is: %d\n', rO)













