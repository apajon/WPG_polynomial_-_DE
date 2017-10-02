function [A]=compute_repartition_matrix_modified(tpassage,nbphases,cutting,type_phase)

A=zeros(6*nbphases+6,1);

% %zmp1 init
% dt=tpassage(2)-tpassage(1);
% dt=dt*cutting;
% m=[1 0 0 0 0 0;
%     0 1 0 0 0 0;
%     0 0 2 0 0 0;
%     1 dt dt^2 dt^3 dt^4 dt^5;
%     0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
%     0 0 2 6*dt 12*dt^2 20*dt^3];
% % m=inv(m);
% m=m\[0;0;0;0.5;0;0];
% A(1:6,1)=m;
% 
% %decoupage de zmp1 init en deux parties
% dt=tpassage(2)-tpassage(1);
% dt=dt*(1-cutting);
% m=[1 0 0 0 0 0;
%     0 1 0 0 0 0;
%     0 0 2 0 0 0;
%     1 dt dt^2 dt^3 dt^4 dt^5;
%     0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
%     0 0 2 6*dt 12*dt^2 20*dt^3];
% % m=inv(m);
% m=m\[0.5;0;0;1;0;0];
% A(7:12,1)=m;

j=0;
for i=2:nbphases-1
    if(type_phase(i)~=0)
        j=j+1;
        dt=tpassage(i+1)-tpassage(i);
        m=[1 0 0 0 0 0;
           0 1 0 0 0 0;
           0 0 2 0 0 0;
           1 dt dt^2 dt^3 dt^4 dt^5;
           0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
           0 0 2 6*dt 12*dt^2 20*dt^3];
%         m=inv(m);
        m=m\[0;0;0;1;0;0];
        A(7+(j)*6:12+(j)*6,1)=m;
    end
end

% %zmp1 fin
% dt=tpassage(end)-tpassage(end-1);
% dt=dt*cutting;
% m=[1 0 0 0 0 0;
%     0 1 0 0 0 0;
%     0 0 2 0 0 0;
%     1 dt dt^2 dt^3 dt^4 dt^5;
%     0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
%     0 0 2 6*dt 12*dt^2 20*dt^3];
% % m=inv(m);
% m=m\[0;0;0;0.5;0;0];
% A(end-11:end-6,1)=m;
% 
% %decoupage de zmp1 fin en deux parties
% dt=tpassage(end)-tpassage(end-1);
% dt=dt*(1-cutting);
% m=[1 0 0 0 0 0;
%     0 1 0 0 0 0;
%     0 0 2 0 0 0;
%     1 dt dt^2 dt^3 dt^4 dt^5;
%     0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
%     0 0 2 6*dt 12*dt^2 20*dt^3];
% % m=inv(m);
% m=m\[0.5;0;0;1;0;0];
% A(end-5:end,1)=m;

end