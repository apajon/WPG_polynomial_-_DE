%Compute the a vector with ZMP polynomial coeff
function [A B]=compute_coeff_zmp1_gradient2(tpassage,psa_zmp1init,psa_zmp1fin,cutting)
nbpoly=length(tpassage)-1;
nbpoly1=(length(tpassage)-1-2)/3+2;
g=[];


m22=zeros(6,nbpoly1-3);

%zmp1 init
dt=tpassage(2)-tpassage(1);
dt=dt*cutting;
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 2 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 2 6*dt 12*dt^2 20*dt^3];
m=inv(m);
m0=zeros(6,0);
m1=m(1:6,1:3);
m2=zeros(6,(nbpoly)*3);

m3=zeros(6,0);
m4=m(1:6,4:6);
m5=zeros(6,3*(nbpoly1-1)+3+3);

g=[g;m0 m1 m2 m22 m3 m4 m5];

%decoupage de zmp1 init en deux parties
dt=tpassage(2)-tpassage(1);
dt=dt*(1-cutting);
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 2 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 2 6*dt 12*dt^2 20*dt^3];
m=inv(m);
m0=zeros(6,0);
m1=zeros(6,0);
m2=zeros(6,(nbpoly)*3+3);

m3=zeros(6,0);
m4=m;
m5=zeros(6,3*(nbpoly1-1)+3);

g=[g;m0 m1 m2 m22 m3 m4 m5];

for i=2:nbpoly-1
    dt=tpassage(i+1)-tpassage(i);
        m=[1 0 0 0 0 0;
           0 1 0 0 0 0;
           0 0 2 0 0 0;
           1 dt dt^2 dt^3 dt^4 dt^5;
           0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
           0 0 2 6*dt 12*dt^2 20*dt^3];
    m=inv(m);

    m0=zeros(6,3*(i-1));
    m1=m(1:6,1:3);
    m2=zeros(6,(nbpoly)*3-(i-1)*3);
    
    m3=zeros(6,(i+1-mod(i+1,3))+3);
    m4=m(1:6,4:6);
    m5=zeros(6,(nbpoly1-1)*3-(i+1-mod(i+1,3))+3);
    
    g=[g;m0 m1 m2 m22 m3 m4 m5];
end

%zmp1 fin
dt=tpassage(end)-tpassage(end-1);
dt=dt*cutting;
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 2 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 2 6*dt 12*dt^2 20*dt^3];
m=inv(m);

m0=zeros(6,(nbpoly)*3-3);
m1=m(1:6,1:3);
m2=zeros(6,3);

m3=zeros(6,3*(nbpoly1-1)+3);
m4=m(1:6,4:6);
m5=zeros(6,3);

g=[g;m0 m1 m2 m22 m3 m4 m5];

%decoupage de zmp1 fin en deux parties
dt=tpassage(end)-tpassage(end-1);
dt=dt*(1-cutting);
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 2 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 2 6*dt 12*dt^2 20*dt^3];
m=inv(m);

m0=zeros(6,(nbpoly)*3-3);
m1=zeros(6,0);
m2=zeros(6,3+3);

m3=zeros(6,3*(nbpoly1-1)+3);
m4=m;
m5=zeros(6,0);

g=[g;m0 m1 m2 m22 m3 m4 m5];

A=[g(:,4:(nbpoly)*3+(nbpoly1-3)) zeros(size(g,1),4) g(:,(nbpoly)*3+(nbpoly1-3)+1+3:end-3)];
B=[g(:,1:3)]*psa_zmp1init+[g(:,end-2:end)]*psa_zmp1fin;
% A=[g(:,4:(nbpoly-1)*3+nbpoly1*3) g(:,(nbpoly-1)*3+nbpoly1*3+5:end)];
% B=[g(:,1:3) g(:,(nbpoly-1)*3+nbpoly1*3+1:(nbpoly-1)*3+nbpoly1*3+3)]*[psa_zmpinit;psa_zmpfin];
% A=[g(:,4:(nbpoly-1)*3) g(:,(nbpoly-1)*3+7:size(g,2)-3)];
% B=[g(:,1:3) g(:,(nbpoly-1)*3+1:(nbpoly-1)*3+6) g(:,size(g,2)-2:size(g,2))]*[psa_zmpinit;psa_zmpfin;psa_zmp1init;psa_zmp1fin];

end