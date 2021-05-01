% syms IL1 VPV IL2 VO IPV 
% syms L1 L2 CPV CO R U TS IPV_T_1
% EQ1 = VPV/(2*L1) * U + VPV/(2*L1) - VO/(2*L1)+ VO/(2*L1)*U == 0;
% 
% EQ2 = -1*IL1/CPV*U + IPV/CPV - IL1/CPV == 0;
% 
% EQ3 = VPV/(2*L2)*U + VPV/(2*L2) - VO/(2*L2) + VO/(2*L2)*U == 0;
% 
% EQ4 = -VO/(R*CO) == 0;
% 
% EQ5 = (IPV - IPV_T_1) / TS == 0;
% 
% [A,B] = equationsToMatrix([EQ1, EQ2, EQ3, EQ4, EQ5], [IL1, VPV, IL2, VO, IPV])
% 
% X = linsolve(A,B)


syms IL1  Vo Vcpv IL2
syms L1  Rpv R Co Cpv Vpv U L2
EQ1 = Vcpv/(2*L1)*U -Vo/(2*L1) + Vcpv/(2*L1) + Vo/(2*L1)*U == 1;

 %EQ2 = Vcpv/(2*L2)*U -Vo/(2*L2) + Vcpv/(2*L2) + Vo/(2*L2)*U == 0.1;

EQ3 = IL2/Co - Vo/(R*Co) - IL2/(Co)*U == 3;

EQ4 = -IL2/Cpv*U + Vpv/(Rpv*Cpv) - Vcpv/(Rpv*Cpv) - IL1/(Cpv) == 2;


[A,B] = equationsToMatrix([EQ1, EQ3, EQ4], [IL1, Vo, Vcpv])

X = linsolve(A,B)