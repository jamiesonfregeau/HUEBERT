function [qS1, qS2, qS3] = voltage2angle(Q1,Q2,Q3)
%Matthew Ebert; 2021-05-25
%Hewbert arm position
%give value in mm


%constants


%Dimension of the robot arm; names based on part names in solidworks
%assembly TECHRAM_MK3.SLDASM
OG = 80 /1000; %m

L1 = 255/ 1000; %m
CL1 = 141.42136/1000; %mxx
L1m = 110/1000; %m

L2 = 260.0 / 1000; %m
CL2 = 280.44607 /1000; %m
L2e = 80 /1000; %m

OL=109.37002/1000; %m
OX = 105.50000 /1000; %m
OY = 28.83664 /1000; %m


qO=0.26681587; %rads
%modify values based on arm Dimensions

Q1 = 2.41 - Q1*(2.41-0.26)/1024 ;
Q2 = 0.35+ Q2*(2.63-0.35)/1024;
%Finding OG2 position

R2 = sqrt(L1^2 +L2e^2 - 2*L1*L2e*cos(pi-Q2));
q_L1_OL2e = acos( (-(L2e)^2  + (R2)^2 + (L1)^2)/(2*R2*L1));
R1 =  sqrt(OL^2 + R2^2 - 2*OL*R2*cos(pi-Q1 - q_L1_OL2e));
q2_beta =  acos( ((OG)^2  - (CL2)^2 + (R1)^2)/(2*OG*R1));
q2_alpha =  acos( ((OL)^2  - (R2)^2 + (R1)^2)/(2*OL*R1));

qS2 = q2_alpha + q2_beta;%Angle of OG2 referenced to the line between center of planetary gear and pivot of L1

%Find OG1 position
q1_theta = pi-Q1;
O_L1m = sqrt(L1m^2 +OL^2 - 2*L1m*OL*cos(q1_theta));
q1_beta =  acos( ((OG)^2  - (CL1)^2 + (O_L1m)^2)/(2*OG*O_L1m));
q1_alpha =  acos( ((OL)^2  - (L1m)^2 + (O_L1m)^2)/(2*OL*O_L1m));

qS1 = q1_beta+ q1_alpha; %Angle of OG1 referenced to the line between center of planetary gear and pivot of L1


qS3 = Q3; %Angle of assembly reference to front.


Q = [qS1; qS2; qS3];


% k1 = -2*L1m*OG*sin(theta_L1x0);
% k2 = 2*OG*(L0-L1m*cos(theta_L1x0));
% k3 = L0^2+L1m^2-CL1^2+OG^2-2*L0*L1m*cos(theta_L1x0);
%q2 =2*pi+ 2*atan2(-k1-sqrt(k1^2+k2^2-k3^2),k3-k2);
%A = -2*L2*y;
%B1 = -2*L2*(x-L0/2);
%B2 = -2*L2*(x+L0/2);
%C1 = x^2+y^2+(L0/2)^2+L2^2-L1^2-L0*x;
%C2 = x^2+y^2+(L0/2)^2+L2^2-L1^2+L0*x;
%q1 = 2*pi-2*atan((-A+sqrt((A)^2+(B1)^2-(C1)^2))/(C1-B1));
%q2 = 2*atan((-A-sqrt(A^2+B2^2-C2^2))/(C2-B2));
%q1 = q1*180/pi;
%q2=q2*180/pi;
%alpha1 = acos( ((a1)^2  - (a2)^2 + (r2)^2)/(2*a4*r2));
%beta1 = atan2(ye, -(L0-xe));
%Based on x and y calculate closed form solution 
%sub solutions
%A = -2*L2*ye;
%B1 = -2*L2*(xe-L0/2);
%C1 = xe^2 +ye^2+(L0/2)^2+L2^2-L1^2-L0*xe;

%B2 = -2*L2*(xe+L0/2);
%C2 = (xe)^2 +(ye)^2   +  (L0/2)^2  +  L2^2  -  L1^2  +  L0*(xe);
%closed form solution
%q1 = 2*atan((-A+(A^2+B1^2-C1^2)^(1/2))/(C1-B1));
%q2 = 2*atan((-A -((A)^2+(B2)^2-(C2)^2)^(1/2))/(C2-B2));
%q1 = acos(((r^2)+(L1^2)-(L2^2))/(2*r*L1))+acos(x/r);
return
end

