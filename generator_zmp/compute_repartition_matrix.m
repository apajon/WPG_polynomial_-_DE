function [A]=compute_repartition_matrix(tpassage,cutting)
nbpoly=length(tpassage)-1;
g=[];

%%%cutting fcom1 init in two%%%
dt=tpassage(2)-tpassage(1);
dt=dt*cutting;
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 2 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 2 6*dt 12*dt^2 20*dt^3];
%m=inv(m);
m=m\[0.5;0;0;0.6;0;0];
g=[g;m];

%second part
dt=tpassage(2)-tpassage(1);
dt=dt*(1-cutting);
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 2 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 2 6*dt 12*dt^2 20*dt^3];
% m=inv(m);
m=m\[0.6;0;0;0;0;0];
g=[g;m];


for i=2:nbpoly-1
    dt=tpassage(i+1)-tpassage(i);
    m=[1 0 0 0 0 0;
       0 1 0 0 0 0;
       0 0 2 0 0 0;
       1 dt dt^2 dt^3 dt^4 dt^5;
       0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
       0 0 2 6*dt 12*dt^2 20*dt^3];
    %m=inv(m);
    m=m\[1;0;0;0;0;0];

    g=[g;m];
end

%%%cutting fcom1 fin in two%%%
dt=tpassage(end)-tpassage(end-1);
dt=dt*(1-cutting);
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 2 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 2 6*dt 12*dt^2 20*dt^3];
% m=inv(m);
m=m\[1;0;0;0.40;0;0];
g=[g;m];

%second part
dt=tpassage(end)-tpassage(end-1);
dt=dt*cutting;
m=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 2 0 0 0;
    1 dt dt^2 dt^3 dt^4 dt^5;
    0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
    0 0 2 6*dt 12*dt^2 20*dt^3];
% m=inv(m);
m=m\[0.40;0;0;0.5;0;0];
g=[g;m];

A=g;
% A=g(:,4:size(g,2)-3);
% B=[g(:,1:3) g(:,size(g,2)-2:size(g,2))]*[psa_zmpinit;psa_zmpfin];

end