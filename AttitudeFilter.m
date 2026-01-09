function [P,quat,omega_bias]=AttitudeFilter(P_,quat_,omega_bias_,omega_b,y_acc,y_mag,ST, g_n, B_n,Var_gyro, Var_acc, Var_mag)

pw=quat_(1);
px=quat_(2);
py=quat_(3);
pz=quat_(4);

% Correction elements
R =  [pw^2+px^2-py^2-pz^2      2*(px*py-pw*pz)       2*(px*pz+pw*py);
            2*(px*py+pw*pz)  pw^2-px^2+py^2-pz^2       2*(py*pz-pw*px);
            2*(px*pz-pw*py)      2*(py*pz+pw*px)   pw^2-px^2-py^2+pz^2];

h1=-R*g_n;
h2=R*B_n;

H = H_attitude(ST,g_n(3),B_n(1),B_n(2),B_n(3),omega_b(1),omega_b(2),omega_b(3),...
    omega_bias_(1),omega_bias_(2),omega_bias_(3),pw,px,py,pz);

V = [ Var_acc*eye(3) , zeros(3,3); zeros(3,3) , Var_mag*eye(3)];

S=H*P_*H'+V;

L=P_*H'/S;

x_ = [quat_; omega_bias_];

% Correction
x = x_ + L*[y_acc - h1; y_mag - h2];
P = (eye(size(H,2))-L*H)*P_*(eye(size(H,2))-L*H)'+L*V*L';

% Prediction elements
quat = x(1:4);
omega_bias = x(5:7);

phi_vec=(omega_b - omega_bias)*ST;
phi = vecnorm(phi_vec);
u = phi_vec/phi;
quat_omega = [cos(phi/2); u*sin(phi/2)];

qw=quat_omega(1);
qx=quat_omega(2);
qy=quat_omega(3);
qz=quat_omega(4);

pw=quat(1);
px=quat(2);
py=quat(3);
pz=quat(4);

F = F_attitude(ST,omega_b(1),omega_b(2),omega_b(3),...
    omega_bias(1),omega_bias(2),omega_bias(3),pw,px,py,pz);

D = D_attitude(ST,omega_b(1),omega_b(2),omega_b(3),...
    omega_bias(1),omega_bias(2),omega_bias(3),pw,px,py,pz);

a = 5;

Q = a*Var_gyro;

% Prediction
quat = [pw*qw - px*qx - py*qy - pz*qz;
        pw*qx + px*qw + py*qz - pz*qy;
        pw*qy - px*qz + py*qw + pz*qx;
        pw*qz + px*qy - py*qx + pz*qw];

P = F*P*F'+ D*Q*D';

end