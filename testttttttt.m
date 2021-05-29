% Continuous Time Small Signal State Space
% Compact form
syms Cpv Cout L1 L2 Rout Rpv RL1 RL2 Vpv u_e Ts

Ac =[(-1/(Cpv*Rpv)) (-1/(Cpv)) (-u_e/(Cpv)) 0 ;
    (1/(L1+L2) + L2*u_e/(L1*(L1+L2)))  (-RL1/(L1+L2) -RL1*L2*u_e/(L1*(L1+L2)))  (-RL2/(L1+L2) + RL2*u_e/(L1+L2)) (-1/(L1+L2) + u_e/(L1+L2));
    (1/(L1+L2) + L1*u_e/(L2*(L1+L2)))  (-RL1/(L1+L2) + RL1*u_e/(L1+L2)) (-RL2/(L1+L2)-RL2*L1*u_e/(L2*(L1+L2))) (-1/(L1+L2) + u_e/(L1+L2));
    0 0 (1/(Cout) - u_e/(Cout)) (-1/(Rout*Cout))];

%VCpv = (L2*RL1*Rout*Vpv*u_e^2 - 2*L2*RL1*Rout*Vpv*u_e + L1*RL1*RL2*Vpv + L2*RL1*RL2*Vpv + L2*RL1*Rout*Vpv)/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);
%IL1 = (Vpv*(L2*RL2 - L1*Rout + L2*Rout + L1*RL2*u_e + 2*L1*Rout*u_e - 2*L2*Rout*u_e - L1*Rout*u_e^2 + L2*Rout*u_e^2))/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);
%IL2 = (RL1*Vpv*(L1 + L2*u_e))/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);
%VCout = -(RL1*Rout*Vpv*(L1 + L2*u_e)*(u_e - 1))/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);

VCpv = (L2*RL1*Rout*Vpv*u_e^2 - 2*L2*RL1*Rout*Vpv*u_e + L1*RL1*RL2*Vpv + L2*RL1*RL2*Vpv + L2*RL1*Rout*Vpv)/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);
IL1 = (Vpv*(L2*RL2 - L1*Rout + L2*Rout + L1*RL2*u_e + 2*L1*Rout*u_e - 2*L2*Rout*u_e - L1*Rout*u_e^2 + L2*Rout*u_e^2))/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);
IL2 =  (RL1*Vpv*(L1 + L2*u_e))/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);
VCout =  -(RL1*Rout*Vpv*(L1 + L2*u_e)*(u_e - 1))/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);
 
% B=[ -IL2/Cpv;
%     L2*VCpv/(L1*(L1+L2))-RL1*L2*IL1/(L1*(L1+L2))+RL2*IL2/(L1+L2)+VCout/(L1+L2);
%     L1*VCpv/(L2*(L1+L2))+RL1*IL1/(L1+L2)-RL2*L1*IL2/(L2*(L1+L2))+VCout/(L1+L2);
%      -IL2/Cout];


Bc = [1/(Cpv*Rpv);
    0;
    0;
    0];



 %outputs = states
 %Cc1 = eye(4);
 %change to outputs =  Vo only
 %Cc1(1,1) = 0; Cc1(2,2) = 0; Cc1(4,4) = 0;
 
 Cc = [0 0 0 1];
 
 %empty D matrix
 Dc = zeros(size(Cc,1), size(Bc,2));

   AAAA = expm(Ac*Ts);
%  states = {'IL1' 'IL2' 'VO' 'VCPV'};
%  inputs = {'u'};
%  outputs = {'VO'};
%  
%  sysc = ss(Ac,Bc,Cc,Dc,'statename', states, 'inputname', inputs, 'outputname', outputs);
%  %sysc = ss(Ac,Bc,Cc,Dc);
%  eig(Ac)
%  step(sysc)
%  
%  %Convert State Space from Continous to Discrete Using Zero Order Hold
%  sys_d = c2d(sysc,Ts, 'zoh')
%  
