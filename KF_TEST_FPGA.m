u_Vpv = 35;
Vout = 80;
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


Vpv = u_Vpv;
y = Vout;
u = u_Vpv;

Ts = 15e-6;

DC = .5; 

% Continuous Time Small Signal State Space
%Aol = [0.943784461257301,-0.0559330122345799,-0.0279395312339836,5.29606701473712e-05;0.00363447688363775,0.996152021590196,-0.00131615408875196,-0.00124644633684318;0.00363447688363775,-0.00135110080726459,0.996186968308708,-0.00124644633684318;5.29606701473712e-05,-1.90030480999995e-05,0.0287831492829427,0.999405251934562];
Ac =[(-1/(Cpv*Rpv)) (-1/(Cpv)) (-DC/(Cpv)) 0 ;
    (1/(L1+L2) + L2*DC/(L1*(L1+L2)))  (-RL1/(L1+L2) -RL1*L2*DC/(L1*(L1+L2)))  (-RL2/(L1+L2) + RL2*DC/(L1+L2)) (-1/(L1+L2) + DC/(L1+L2));
    (1/(L1+L2) + L1*DC/(L2*(L1+L2)))  (-RL1/(L1+L2) + RL1*DC/(L1+L2)) (-RL2/(L1+L2)-RL2*L1*DC/(L2*(L1+L2))) (-1/(L1+L2) + DC/(L1+L2));
    0 0 (1/(Cout) - DC/(Cout)) (-1/(Rout*Cout))];
Bc = [1/(Cpv*Rpv);
    0;
    0;
    0];
 Cc = [0 0 0 1];
 
 Dc = zeros(size(Cc,1), size(Bc,2));
 
 %discretize my state space model using ZOH method
 Aol = eye(4) + Ac*Ts;
 Bol = Bc*Ts;
 Col = Cc;
 Dol = Dc;
 
 
%Bol = [0.0560566260169120;0.000105941817191590;0.000105941817191590;1.02384484238660e-06];

%Col = [0,0,0,1];

%Dol = 0;
 %Design the Observer System:
 

 %covar matricies, process q, measurement r -- must be positive numbers
 Qcov = diag(.0001*ones(1,4)); %modify the numbers to change how it filters noise
 Rcov = diag(.01*ones(1,1)); %modify the numbers to change how it filters noise
 
xhat_t_1 = [.1; .1; .1; .1];

P_t_1 = [.1 .1 .1 .1;
            .1 .1 .1 .1;
            .1 .1 .1 .1;
            .1 .1 .1 .1];

%prediction stage
xhat_pred = Aol*xhat_t_1+Bol*u;
P_pred = Aol*P_t_1*transpose(Aol)+ Qcov;

%update stage
y_update = y - Col*xhat_pred;
S_update = Col*P_pred*transpose(Col) + Rcov;
K_update = P_pred*transpose(Col)*(1/S_update);
xhat_update = xhat_pred + K_update*y_update;
P_update = (eye(4) - K_update*Col)*P_pred;

%Est_Vpv =  xhat_update(4)*(1-DC)/(1+DC) + .35;
%Vpv_out = Est_Vpv;

%increase DC by .01:
DC = DC + .0001;
if(DC > 1)
    DC = 1;
end
Ac =[(-1/(Cpv*Rpv)) (-1/(Cpv)) (-DC/(Cpv)) 0 ;
    (1/(L1+L2) + L2*DC/(L1*(L1+L2)))  (-RL1/(L1+L2) -RL1*L2*DC/(L1*(L1+L2)))  (-RL2/(L1+L2) + RL2*DC/(L1+L2)) (-1/(L1+L2) + DC/(L1+L2));
    (1/(L1+L2) + L1*DC/(L2*(L1+L2)))  (-RL1/(L1+L2) + RL1*DC/(L1+L2)) (-RL2/(L1+L2)-RL2*L1*DC/(L2*(L1+L2))) (-1/(L1+L2) + DC/(L1+L2));
    0 0 (1/(Cout) - DC/(Cout)) (-1/(Rout*Cout))];
Aol_increase = eye(4) + Ac*Ts;
%predict 1 timestep ahead on
xhat_plus_on = Aol_increase*xhat_update + Bol*u;
Ipv_plus_on = (u - xhat_plus_on(1))/Rpv;
%decrease DC by .01
DC = DC - 2*.0001;
if(DC < 0) 
    DC = 0; end
Ac =[(-1/(Cpv*Rpv)) (-1/(Cpv)) (-DC/(Cpv)) 0 ;
    (1/(L1+L2) + L2*DC/(L1*(L1+L2)))  (-RL1/(L1+L2) -RL1*L2*DC/(L1*(L1+L2)))  (-RL2/(L1+L2) + RL2*DC/(L1+L2)) (-1/(L1+L2) + DC/(L1+L2));
    (1/(L1+L2) + L1*DC/(L2*(L1+L2)))  (-RL1/(L1+L2) + RL1*DC/(L1+L2)) (-RL2/(L1+L2)-RL2*L1*DC/(L2*(L1+L2))) (-1/(L1+L2) + DC/(L1+L2));
    0 0 (1/(Cout) - DC/(Cout)) (-1/(Rout*Cout))];
DC = DC + .0001;
Aol_decrease = eye(4) + Ac*Ts;
%predict 1 timestep ahead off
xhat_plus_off = Aol_decrease*xhat_update + Bol*u;
Ipv_plus_off = (u - xhat_plus_off(1))/Rpv;

%update function output and variables
P_t_1 = P_update;
xhat_t_1 = xhat_update;
xhat = xhat_update;
Ipv = (u - xhat(1))/Rpv;
Ipv = Ipv;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if (Ipv > 10)
%     Ipv = 1;
% end
% 




Ts = 15e-6; %sample rate
L = 3e-3; %inductance

Vpv_t_1 = 0;
Ipv_t_1 = 0;
Dpv_t_1 = 0;
Iref = 0;
Dpv = Iref;
delta_V = Vpv - Vpv_t_1;
delta_I = Ipv - Ipv_t_1;
delta_D = Dpv - Dpv_t_1;


%fixed vs adaptive
scale_factor_C = 10; %increase for larger output increments/swings
 Z =.001; %fixed 
%Z = scale_factor_C * abs(delta_I * delta_D); %adaptive

% custom adaptive to fix issues with no change causing 0 step size
 adapZ = scale_factor_C * abs(delta_I * delta_D);
% if(adapZ > .1)
%     Z = .1;
% elseif(adapZ > .001)
%     Z = adapZ;
% else
%     Z = .001;
% end

Ipred_1 = Ipv_plus_on;
Ipred_0 = Ipv_plus_off;
%Ipred_1 = Ipv + 2*(Ts/L) * Vpv; %MPC prediction for switch = 1
%Ipred_0 = Ipv + (Ts/(2*L)* (Vpv - Vout)); %MPC prediction for switch = 0 

if delta_V == 0 %Inc. Cond. Algo to determine Iref
    if delta_I == 0 
        Iref = Dpv; choice = 0;
    else
        if delta_I > 0
            Iref = Dpv - Z; choice = 1;
        else
            Iref = Dpv + Z; choice = 2;
        end
    end
else
    if (Ipv + (delta_I/delta_V) * Vpv) == 0
        Iref = Dpv; choice = 3;
    else
        if (Ipv + (delta_I/delta_V) * Vpv) > 0
            Iref = Dpv - Z; choice = 4;
        else
            Iref = Dpv + Z; choice = 5;
        end
    end
end

G1 = abs(Iref - Ipred_1); %Optimize MPC by applying Iref
G0 = abs(Iref - Ipred_0);
if G1 < G0
    DC = DC + .0001;
else
    DC = DC - .0001;
end

if (DC < 0)
    DC = 0;
end
if (DC > 1)
    DC = 1;
end

if (Iref < 0)
    Iref = .001;
end
%test = Ipv + (delta_I/delta_V) * Vpv;
test =  delta_V;
%update static variables here:
Vpv_t_1 = Vpv; %previous Vpv Sample
Ipv_t_1 = Ipv; %previous Ipv Sample
Dpv_t_1 = Dpv; % Iref (output) value from what is now 2 timesteps ago
Dpv = Iref;  %Previous / Most recent Iref (output) Calc

y = DC;

%debug purposes:
debug1 = Iref;
debug2 = delta_I/delta_V;