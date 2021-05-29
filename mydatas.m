syms VC IL E L C R u_e

EQ1 = -VC/L + VC*u_e/L + E/L == 0;
EQ2 =  IL/C - IL*u_e/C -VC/(R*C) == 0;


[A,B] = equationsToMatrix([EQ1, EQ2], [IL, VC]);
X = linsolve(A,B)