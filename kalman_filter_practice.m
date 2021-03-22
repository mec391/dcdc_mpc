%kalman filter practice

Ac = [-0.026 .074 -.804 -9.8 0;
    -0.242 -2.017 73.297 -.105 -.001;
     0.003 -.135 -2.941 0 0;
     0 0 1 0 0;
     -.011 1 0 -75 0];
 
 Bc = [4.594 0;
     -0.0004 -13.75;
     0.0002 -24.410;
     0 0;
     0 0];
 
 Cc = eye(5);
 
 Dc = zeros(size(Cc,1), size(Bc,2));
 
 states = {'u' 'w' 'q' '\theta' 'h'};
 inputs = {'delta_t' 'delta_e'};
 outputs = states;
 
 sysc = ss(Ac,Bc,Cc,Dc,'statename', states, 'inputname', inputs, 'outputname', outputs);
 step(sysc)
 
 %sampling time
 dT = .05; %20Hz
 sys_d = c2d(sysc,dT, 'zoh'); %zero order hold
 
 %discrete matricies
 Aol = sys_d.A;
 Bol = sys_d.B;
 Col = sys_d.C;
 Dol = sys_d.D;
 
 %observer design:
 
 %G and H
 G = Cc; % 5 states, 5 disturbances
 H = zeros(5,5);
 
 %covar matricies, process q, measurement r
 Qcov = diag(0.15*ones(1,5)); %tweak the numbers to change how it filters noise
 Rcov = diag(0.05*ones(1,5)); %tweak the numbers to change how it filters noise
 
 %ss with disturbances
 sys_kf = ss(Aol,[Bol G], Col, [Dol H], dT);
 
 %obtain L and P assuming w and V are uncorrellated
 [kest,L,P] = kalman(sys_kf, Qcov, Rcov, 0);
 
 %L_bar must = L
 L_bar = (Aol * P * Col)/(Col*P*Col'+Rcov);
 
 Error = norm(abs(L_bar - L));
 
 %get stability
 Acb = Aol - L*Col;
 eig(Acb)