function y = ekf_state_transition(x, w, u)

X=u(1:4);
Vpv=u(5);
u=u(6);

xx = x;
ww = w;
%5/10/2021 UPDATED
% parameters:


Cpv=260e-6;
Cout=260e-6;
L1=3e-3;
L2=3e-3;
Rout=100;
Rpv = 1;
RL1 = 1;
RL2 = 1;

% Compact form
A=[ -1/(Cpv*Rpv) -1/Cpv 0 0 ;
    1/(L1+L2) -RL1/(L1+L2) -RL2/(L1+L2) -1/(L1+L2) ;
    1/(L1+L2) -RL1/(L1+L2)  -RL2/(L1+L2) -1/(L1+L2);
    0 0 1/Cout -1/(Rout*Cout)];

B=[0 0 -1/(Cpv) 0;
    L2/(L1*(L1+L2)) -RL1*L2/(L1*(L1+L2)) RL2/(L1+L2) 1/(L1+L2);
     L1/(L2*(L1+L2)) RL1/(L1+L2) -RL2*L1/(L2*(L1+L2)) 1/(L1+L2);
    0  0 -1/(Cout) 0 ];
     
     
b=[0; 0; 0; 0];

d=[Vpv/(Cpv*Rpv); 0; 0; 0];

y = A*X+(B*X+b)*u+d;

