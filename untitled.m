%% Cylinder information
clc;clear;close all;

% Cylinder 1 and 2 have equal lengths.
x_min1 = 2.333; % [m] Tube Length of cylinder 
x_min2 = 2.846; % [m] Piston Length of cylinder 

% Cylinder Tube/Rod Diameters
d_p1 = 0.28; % [m] Piston Diameter of cylinder 1
d_r1 = 0.2;  % [m] Rod Diameter of cylinder 1
d_p2 = 0.25; % [m] Piston Diameter of cylinder 2
d_r2 = 0.18; % [m] Rod Diameter of cylinder 2
% masses 
m_cyl1 = 1208; % [g]
m_cyl2 = 1208; % [g]
m_pist1 = 818; % [g]
m_pist2 = 818; % [g]

V_cyl_p1=1/5 * x_min2 * d_p1^2 *pi + 4/5 *x_min2 *d_r1^2 *pi; % volume of piston cylinder 1

% centre of mass equation: 
% xCM = x1*m1 + x2*m2 / sum(m1+m2)
xCM_cyl_p1=1.31;    %[m] - found using model from solidworks
xCM_cyl_t1=x_min1/2+xCM_cyl_p1 %[m]
xCM_cyl_p2=1.31;    %[m] - found using model from solidworks
xCM_cyl_t2=x_min1/2 %[m]

%% Lasse
%% Auxiliary Elements
% Simulation Time Step
T_sim = 1/2000;
% Gravitational Acceleration
g = 9.82;
%% Cylinder friction parameters   
Bv1 = 40000;
Bv2 = Bv1;
%% Knuckle Boom Crane Parameters
% Dimensional Vector
rcm1 = [6.3957; 0.2815];
rcm2 = [4.1947; 0.7957];
rCD = [4.3736; -0.4625];
rCG = [9.7657; -0.4358];
rCF = [13.6883; 0.25e-2];
rFH = [1.6705; 0.6845];
rFJ = [9.4988; 0];
rBC = [1.12; -2.3];
rAC = [0; -7.3];
rAB = [1.12; 5];
rDF = [9.3147; .465];
rHJ = [7.88283; -0.6845];
rFG = [-3.9227; -0.4383];
% Vector Lengths
L_CD = norm(rCD);
L_CG = norm(rCG);
L_CCM1 = norm(rcm1);
L_CF = norm(rCF);
L_FH = norm(rFH);
L_FCM2 = norm(rcm2);
L_FCM2x = rcm2(1);
L_FCM2y = rcm2(2);
L_CCM1x = rcm1(1);
L_CCM1y = rcm1(2);
L_FHx = rFH(1);
L_FHy = rFH(2);
L_FGx = rFG(1);
L_FGy = rFG(2);
L_FJx = rFJ(1);
L_FJy = rFJ(2);
L_HJx = rHJ(1);
L_HJy = rHJ(2);
L_ACx = rAC(1);
L_ACy = rAC(2);
L_ABx = rAB(1);
L_ABy = rAB(2);
L_BCx = rBC(1);
L_BCy = rBC(2);
L_CGx = rCG(1);
L_CGy = rCG(2);
L_CDx = rCD(1);
L_CDy = rCD(2);
L_CFx = rCF(1);
L_CFy = rCF(2);
L_DFx = rDF(1);
L_DFy = rDF(2);
L_FJ = norm(rFJ);
L_BC = norm(rBC);
L_AC = norm(rAC);
L_AB = norm(rAB);
L_DF = norm(rDF);
L_HJ = norm(rHJ);
L_FG = norm(rFG);
L_JCM3 = 1.9;
% Relevant Angles
alpha_1 = acos((1/2)*(L_CD^2+L_CF^2-L_DF^2)/(L_CD*L_CF));
alpha_2 = acos((1/2)*(L_FH^2+L_FJ^2-L_HJ^2)/(L_FH*L_FJ));
alpha_4 = acos((1/2)*(-L_AB^2+L_AC^2+L_BC^2)/(L_AC*L_BC));
alpha_6 = acos((1/2)*(L_CF^2-L_CG^2+L_FG^2)/(L_FG*L_CF));
% Cylinder Limits
x_min1 = 2.333;
x_min2 = 2.846;
x_max1 = 2*x_min1;
x_max2 = 2*x_min2;
% Cylinder Piston Strokes
xP1_max = x_max1-x_min1;
xP2_max = x_max2-x_min2;
xP1_ini = 1.9;
xP2_ini = 1.0;
% Initial Cylinder Piston Positions
x_ini1 = xP1_ini+x_min1;%-32/1000+x_min1+TrajectoryCyl1(2,1)+xP1_max/2;
x_ini2 = xP2_ini+x_min2;%200/1000+30/1000+x_min2+TrajectoryCyl2(2,1)+xP2_max/2;
% Initial Joint Angles
phi1_ini = alpha_1+acos((1/2)*(L_BC^2+L_CD^2-x_ini1^2)/(L_BC*L_CD))+alpha_4-(1/2)*pi;
phi2_ini = -alpha_2+acos((1/2)*(L_FG^2+L_FH^2-x_ini2^2)/(L_FG*L_FH))+alpha_6-pi;
phi3_ini = -pi/2-phi1_ini-phi2_ini;
%
xP1_ini1 = -x_min1 + sqrt(-0.2e1 * cos(-phi1_ini + alpha_1 + alpha_4 - pi / 0.2e1) * L_BC * L_CD + L_BC ^ 2 + L_CD ^ 2);
xP2_ini1 = -x_min2 + sqrt(-0.2e1 * cos(phi2_ini + alpha_2 - alpha_6 + pi) * L_FG * L_FH + L_FG ^ 2 + L_FH ^ 2);
% Joint Angle Limits due to Cylinder Limits
phi1_min = alpha_1+acos((1/2)*(L_BC^2+L_CD^2-x_min1^2)/(L_BC*L_CD))+alpha_4-(1/2)*pi;
phi2_min = -alpha_2+acos((1/2)*(L_FG^2+L_FH^2-x_min2^2)/(L_FG*L_FH))+alpha_6-pi;
phi3_min = -inf;
phi1_max = alpha_1+acos((1/2)*(L_BC^2+L_CD^2-x_max1^2)/(L_BC*L_CD))+alpha_4-(1/2)*pi;
phi2_max = -alpha_2+acos((1/2)*(L_FG^2+L_FH^2-x_max2^2)/(L_FG*L_FH))+alpha_6-pi;
phi3_max = inf;
% Masses
m1 = 6000;
m2 = 3300;
m3 = 4000;
m_cyl1 = 1208;
m_cyl2 = 1208;
m_pist1 = 818;
m_pist2 = 818;
% payload Sphere radius
r_load = 0.4;
% Cylinder Tube/Rod Diameters
d_p1 = 0.28;
d_r1 = 0.2;
d_p2 = 0.25;
d_r2 = 0.18;
% Inertias @ CMs
J1 = 107080;
J2 = 22104.;
J3 = (2/5)*m3*r_load^2;
Jcyl1 = (1/12)*m_cyl1*(3*d_p1^2+x_min1^2)+(1/12)*m_pist1*(3*d_r1^2+(x_max1-x_min1)^2);
Jcyl2 = (1/12)*m_cyl2*(3*d_p2^2+x_min2^2)+(1/12)*m_pist2*(3*d_r2^2+(x_max2-x_min2)^2);
% Parameters Related to State Dependent Cylinder+Piston CMs
k_x1 = x_max1-x_min1/2;
c_x1 = m_pist1/(m_cyl1+m_pist1);
k_x2 = x_max2-x_min2/2;
c_x2 = m_pist2/(m_cyl2+m_pist2);
%% Hydraulic Drive Parameters
% Tank Pressure
pT = 0;
% General Bulk Modulii Parameters
patm = 1.013e5;                             
eta_air = 0.7e-2;                            
kappa_air = 1.4;                             
beta_F = 7500e5;                             
% Cylinder Areas
A_A1 = (d_p1/2)^2*pi;
A_B1 = (d_p1/2)^2*pi-(d_r1/2)^2*pi;
A_A2 = (d_p2/2)^2*pi;
A_B2 = (d_p2/2)^2*pi-(d_r2/2)^2*pi;
% Cylinder Area Ratios
alphac1 = A_B1/A_A1;
alphac2 = A_B2/A_A2;
% Pipe/Hose Volumes
VA1ini = 80/1000;
VB1ini = 80/1000;
VA2ini = 80/1000;
VB2ini = 80/1000;
%%

% xP1_var = linspace(0+x_min1,xP1_max+x_min1,1000);
% 
% alpha_3  = zeros(size(xP1_var));
% alpha_7  = zeros(size(xP1_var));
% L_CCM1t  = zeros(size(xP1_var));
% alpha_8  = zeros(size(xP1_var));
% L_CCM1p  = zeros(size(xP1_var));
% alpha_10 = zeros(size(xP1_var));
% 
% for i=1:length(xP1_var)
% alpha_3(i) = acos((L_BC^2 + L_CD^2 - xP1_var(i)^2)/(2*L_BC*L_CD));
% alpha_7(i) = acos((L_CD^2 + xP1_var(i)^2 - L_BC^2)/(2*L_CD*xP1_var(i)));
% L_CCM1t(i) = sqrt(L_CD^2 + xCM_cyl_t1^2 - 2*L_CD*xCM_cyl_t1*cos(alpha_7(i)));
% alpha_8(i) = asin((sin(alpha_7(i)))/L_CCM1t(i) * xCM_cyl_t1);
% L_CCM1p(i) = sqrt(L_CD^2 + xCM_cyl_p1^2 - 2*L_CD*xCM_cyl_p1*cos(alpha_7(i)));
% alpha_10(i) = asin((sin(alpha_7(i)))/L_CCM1p(i)* xCM_cyl_p1);
% end
% phi1_var = linspace(phi1_min,phi1_max,1000);
% 
% figure(1)
% plot(phi1_var,alpha_3)
% title('phi1 var,alpha3')
% 
% figure(2)
% plot(phi1_var,alpha_7)
% title('phi1 var,alpha7')
% 
% figure(3)
% plot(phi1_var,alpha_8)
% title('phi1 var,alpha8')
% 
% figure(4)
% plot(phi1_var,alpha_10)
% title('phi1 var,alpha10')

%%
xP2_var = linspace(0+x_min2,xP2_max+x_min2,1000);
L_HCM2p=x_min2-1.31;    %[m] - found using model from solidworks
% L_HCM2t = x_min2/2;

L_HCM2t = zeros(size(xP2_var));
alpha_10  = zeros(size(xP2_var));
alpha_11  = zeros(size(xP2_var));
alpha_12 = zeros(size(xP2_var));
L_FCM2t  = zeros(size(xP2_var));
L_FCM2p  = zeros(size(xP2_var));

for i=1:length(xP2_var)
L_HCM2t(i)=xP2_var(i)-x_min2/2;
alpha_10(i) = acos((xP2_var(i)^2 + L_FH^2 - L_FG^2)/(2*xP2_var(i)*L_FH));
L_FCM2t(i) = sqrt(L_HCM2t(i)^2 + L_FH^2 - 2*L_HCM2t(i)*L_FH*cos(alpha_10(i)));
L_FCM2p(i) = sqrt(L_HCM2p^2 + L_FH^2 - 2*L_HCM2p*L_FH*cos(alpha_10(i)));
alpha_11(i) = asin(sin(alpha_10(i))/(L_FCM2t(i)) * L_HCM2t(i));
alpha_12(i) = asin(sin(alpha_10(i))/(L_FCM2p(i)) * L_HCM2p);
end
phi2_var = linspace(phi2_min,phi2_max,1000);

figure()
plot(phi2_var,alpha_10)
title('phi2 var,alpha10')

figure()
plot(phi2_var,alpha_11)
title('phi2 var,alpha11')

figure()
plot(phi2_var,alpha_12)
title('phi2 var,alpha12')

% figure()
% plot(phi2_var,alpha_10)
% title('phi1 var,alpha10')
