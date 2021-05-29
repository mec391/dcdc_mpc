Cpv=260e-6;
Cout=260e-6;
L1=3e-3;
L2=3e-3;
Rout=100;
Rpv = 1;
RL1 = 1;
RL2 = 1;

Vpv = 30;
A=[ 0    (-1/L + u_e/L);
   (1/C - u_e/C)  (-1/(C*R))];

IL = E/(R*(u_e-1)^2);
VC = -E/(u_e-1);

B=[0   IL/L;
   -VC/C  0];
u_e = .5;
VCpv = (L2*RL1*Rout*Vpv*u_e^2 - 2*L2*RL1*Rout*Vpv*u_e + L1*RL1*RL2*Vpv + L2*RL1*RL2*Vpv + L2*RL1*Rout*Vpv)/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);
IL1 = (Vpv*(L2*RL2 - L1*Rout + L2*Rout + L1*RL2*u_e + 2*L1*Rout*u_e - 2*L2*Rout*u_e - L1*Rout*u_e^2 + L2*Rout*u_e^2))/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);
IL2 =  (RL1*Vpv*(L1 + L2*u_e))/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);
VCout =  -(RL1*Rout*Vpv*(L1 + L2*u_e)*(u_e - 1))/(L1*RL1*RL2 + L2*RL1*RL2 + L2*RL1*Rout + L2*RL2*Rpv - L1*Rout*Rpv + L2*Rout*Rpv + L2*RL1*Rout*u_e^2 + L2*RL1*Rpv*u_e^2 - L1*Rout*Rpv*u_e^2 + L2*Rout*Rpv*u_e^2 - 2*L2*RL1*Rout*u_e + L1*RL1*Rpv*u_e + L1*RL2*Rpv*u_e + 2*L1*Rout*Rpv*u_e - 2*L2*Rout*Rpv*u_e);
 
