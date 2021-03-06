%Kalman filter design using Small Signal Model of Circuit

%The purpose of this module is to approximate Ipv and Vpv so that a sensor
%is not needed for those variables, it also functions to remove noise and
%could possibly replace the MCP module using its own predictions

%Existing Model assumes Constant Vpv with added series resistor so that
% Ipv = (Vpv - Vcpv)/Rpv
%Also assumes L1 = L2

%Existing state variables: IL1 IL2 Vo Vcpv

%define circuit parameters 

Cpv=260e-6;
Cout=260e-6;
L1=3e-3;
L2=3e-3;
Rout=100;
Rpv = 1;
RL1 = 1;
RL2 = 1;


Vpv = 30;

u_e = .5;

Ts = 15e-6;

% Continuous Time Small Signal State Space
% Compact form
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

 
 states = {'IL1' 'IL2' 'VO' 'VCPV'};
 inputs = {'u'};
 outputs = {'VO'};
 
 sysc = ss(Ac,Bc,Cc,Dc,'statename', states, 'inputname', inputs, 'outputname', outputs);
 %sysc = ss(Ac,Bc,Cc,Dc);
 eig(Ac)
 step(sysc)
 
 %Convert State Space from Continous to Discrete Using Zero Order Hold
 sys_d = c2d(sysc,Ts, 'zoh')
 
 %discrete matricies
 Aol = sys_d.A; %should be new values
 Bol = sys_d.B; %should be new values
 Col = sys_d.C; %should be the same
 Dol = sys_d.D; %should be the same
 
 

 
 [bb0,aa0] = ss2tf(Aol,Bol,Col,Dol)
 %Design the Observer System:
 
 %G and H
 G = eye(4); % 4 states, 4 disturbances
 H = [0 0 0 0]; %not including W noise for output
 
 %covar matricies, process q, measurement r -- must be positive numbers
 Qcov = diag(.000001*ones(1,4)); %tweak the numbers to change how it filters noise
 Rcov = diag(10*ones(1,1)); %tweak the numbers to change how it filters noise
 

 %ss with disturbances
 sys_kf = ss(Aol,[Bol G], Col, [0 H], Ts);

 
 %kest = kalman estimator state space model
 %obtain L and P assuming W and V are uncorrellated
 [kest,L,P] = kalman(sys_kf, Qcov, Rcov, 0);
 
 %L_bar must = L
 %L_bar = (Aol * P * Col)/(Col*P*Col'+Rcov);
 
 %Error = norm(abs(L_bar - L));
 
 %get stability -- must be within unit circle
 Acb = Aol - L*Col;
 eig(Acb)
 rank(obsv(sysc))
 
 
 myt = out.kfdatas(:,1);
 my1 = out.kfdatas(:,2);
 my2 = out.kfdatas(:,3);
 my3 = out.kfdatas(:,4);
 
 plot(myt, my1);
 hold on
% plot(myt, my2);
 plot(myt, my3);