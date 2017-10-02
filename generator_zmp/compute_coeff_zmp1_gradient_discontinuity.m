%Compute the a vector with ZMP polynomial coeff
function [A B]=compute_coeff_zmp1_gradient_discontinuity(tpassage,psa_zmp1init,cutting)
nbphase=length(tpassage)-1;%number of phases
nbstep=(nbphase-2)/3;
%nbparam=(nbphase-2)/3+2;%number of DSP
nbparam=(2*nbstep+2+3)*3+3;
g=[];

%zmp1 init
dt=tpassage(2)-tpassage(1);
dt=dt*cutting;
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 1 6*dt 12*dt^2 20*dt^3];
m=inv(m);
m1=zeros(6,0);
%m2=m;
m3=zeros(6,nbparam-6);

g=[g;m1 m m3];

%decoupage de zmp1 init en deux parties
dt=tpassage(2)-tpassage(1);
dt=dt*(1-cutting);
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 1 6*dt 12*dt^2 20*dt^3];
m=inv(m);
m1=zeros(6,3);
%m2=m;
m3=zeros(6,nbparam-6-3);

g=[g;m1 m m3];

for i=2:nbphase-1
    dt=tpassage(i+1)-tpassage(i);
        m=[1 0 0 0 0 0;
           0 1 0 0 0 0;
           0 0 1 0 0 0;
           1 dt dt^2 dt^3 dt^4 dt^5;
           0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
           0 0 1 6*dt 12*dt^2 20*dt^3];
    m=inv(m);

%     m0=zeros(6,3*(i-1));
%     m1=m(1:6,1:3);
%     m2=zeros(6,(nbpoly)*3-(i-1)*3);
%     
%     m3=zeros(6,(i+1-mod(i+1,3))+3);
%     m4=m(1:6,4:6);
%     m5=zeros(6,3*(nbparam-1)-(i+1-mod(i+1,3))+3);
%     
%     g=[g;m0 m1 m2 m22 m3 m4 m5];
    
    m1=zeros(6,2*(i+1-mod(i+1,3))+3);
    %m2=m
    m3=zeros(6,nbparam-2*(i+1-mod(i+1,3))-6-3);
    
    g=[g;m1 m m3];
end

%zmp1 fin
dt=tpassage(end)-tpassage(end-1);
dt=dt*(1-cutting);
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 1 6*dt 12*dt^2 20*dt^3];
m=inv(m);
m1=zeros(6,nbparam-6-3);
%m2=m;
m3=zeros(6,3);

g=[g;m1 m m3];

%decoupage de zmp1 fin en deux parties
dt=tpassage(end)-tpassage(end-1);
dt=dt*cutting;
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 1 6*dt 12*dt^2 20*dt^3];
m=inv(m);
m1=zeros(6,nbparam-6);
%m2=m;
m3=zeros(6,0);
g=[g;m1 m m3];

A=[g(:,4:end)];
B=[g(:,1:3)]*[psa_zmp1init];
% A=[g(:,4:(nbpoly)*3+(nbparam-3)) g(:,(nbpoly)*3+(nbparam-3)+1+3:end)];
% B=[g(:,1:3)]*[psa_zmp1init];
% A=[g(:,4:(nbpoly-1)*3+nbparam*3) g(:,(nbpoly-1)*3+nbparam*3+5:end)];
% B=[g(:,1:3) g(:,(nbpoly-1)*3+nbparam*3+1:(nbpoly-1)*3+nbparam*3+3)]*[psa_zmpinit;psa_zmpfin];
% A=[g(:,4:(nbpoly-1)*3) g(:,(nbpoly-1)*3+7:size(g,2)-3)];
% B=[g(:,1:3) g(:,(nbpoly-1)*3+1:(nbpoly-1)*3+6) g(:,size(g,2)-2:size(g,2))]*[psa_zmpinit;psa_zmpfin;psa_zmp1init;psa_zmp1fin];

end