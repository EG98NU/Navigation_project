function [pos,vel,P] = PosVelFilter(pos_,vel_,P_,y_UWB,f,R,g_n,ST,Var_acc,Var_uwb,Points)

state_ = [pos_; vel_];

% Correction elements

x = pos_(1); y = pos_(2); z = pos_(3);

h = zeros(size(Points,1),1);
H = zeros(size(Points,1),length(state_));

for k = 1 : size(Points,1)
    h(k) = sqrt( (x - Points(k,1))^2 + (y - Points(k,2))^2 + (z - Points(k,3))^2 );
    H(k,1) = (x - Points(k,1))/h(k); 
    H(k,2) = (y - Points(k,2))/h(k); 
    H(k,3) = (z - Points(k,3))/h(k); 
end

e = y_UWB - h; % innovation

V = Var_uwb*eye(size(Points,1));

S = H*P_*H' + V;

L = P_*H'/S;

% Correction
state = state_ + L*e;
P = (eye(size(H,2)) - L*H)*P_*(eye(size(H,2)) - L*H)' + L*V*L';

% Prediction elements

F = [ eye(3) eye(3)*ST; zeros(3,3) eye(3)];

a=1e4;
D = [ zeros(3,3) ; R*ST];
Q = a*Var_acc;

% Prediction
pos = state(1:3);
vel = state(4:6);

pos = pos + vel*ST;
vel = vel + (R*f + g_n)*ST;

P = F*P*F' + D*Q*D';

end
      
