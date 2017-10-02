%Compute the a vector with in air ankle polynomial coeff
function [A]=compute_coeff_in_air(tpassage,nbphases,type_phase,cutting)

% g=[];
% A=zeros(4*sum(any(type_phase,1))+8,6*(sum(any(type_phase,1))-2)/2+12);
% 
% dt=tpassage(3)-tpassage(2);
% dt=dt*0.5;
% m1=[1 0 0 0;
%    0 1 0 0;
%    1 (dt) (dt)^2 (dt)^3;
%    0 1 2*dt^1 3*dt^2;];
% m1=inv(m1);
% m2=[1 0 0 0;
%    0 1 0 0;
%    1 (dt) (dt)^2 (dt)^3;
%    0 1 2*dt^1 3*dt^2;];
% m2=inv(m2);
% m=[m1 zeros(4,2);
%    zeros(4,2) m2;];
% A(1:8,1:6)=m;
% 
% j=0;
% for i=3:nbphases-2
%     dt=tpassage(i+1)-tpassage(i);
%     if(type_phase(i)==2)
%         m=[1 0 0 0;
%           0 1 0 0;
%           1 (dt) (dt)^2 (dt)^3;
%           0 1 2*dt^1 3*dt^2;];
%         m=inv(m);
%         A(1+j*4+8:4+j*4+8,1+(j-1)/2*6+2+6:4+(j-1)/2*6+2+6)=m;
%         j=j+1;
%     elseif(type_phase(i)==1)
%         m=[1 0 0 0;
%           0 1 0 0;
%           1 (dt) (dt)^2 (dt)^3;
%           0 1 2*dt^1 3*dt^2;];
%       m=inv(m);
%       A(1+j*4+8:4+j*4+8,1+j/2*6+6:4+j/2*6+6)=m;
%       j=j+1;
%     end
% 
% end
% 
% dt=tpassage(end-1)-tpassage(end-2);
% dt=dt*0.5;
% m1=[1 0 0 0;
%    0 1 0 0;
%    1 (dt) (dt)^2 (dt)^3;
%    0 1 2*dt^1 3*dt^2;];
% m1=inv(m1);
% m2=[1 0 0 0;
%    0 1 0 0;
%    1 (dt) (dt)^2 (dt)^3;
%    0 1 2*dt^1 3*dt^2;];
% m2=inv(m2);
% m=[m1 zeros(4,2);
%    zeros(4,2) m2;];
% A(end-7:end,end-5:end)=m;

A=zeros(6*sum(any(type_phase,1))+12,9*(sum(any(type_phase,1))-2)/2+18);

dt=tpassage(3)-tpassage(2);
dt=dt*0.5;
m1=[1 0 0 0 0 0;
   0 1 0 0 0 0;
   0 0 2 0 0 0;
   1 (dt) (dt)^2 (dt)^3 (dt)^4 (dt)^5;
   0 1 2*dt^1 3*dt^2 4*(dt)^3 5*(dt)^4;
   0 0 2 6*dt 12*(dt)^2 20*(dt)^3;];
m1=inv(m1);
m2=[1 0 0 0 0 0;
   0 1 0 0 0 0;
   0 0 2 0 0 0;
   1 (dt) (dt)^2 (dt)^3 (dt)^4 (dt)^5;
   0 1 2*dt^1 3*dt^2 4*(dt)^3 5*(dt)^4;
   0 0 2 6*dt 12*(dt)^2 20*(dt)^3;];
m2=inv(m2);
m=[m1 zeros(6,3);
   zeros(6,3) m2;];
A(1:12,1:9)=m;

j=0;
for i=3:nbphases-2
    dt=tpassage(i+1)-tpassage(i);
    if(type_phase(i)==2)
        m=[1 0 0 0 0 0;
           0 1 0 0 0 0;
           0 0 2 0 0 0;
           1 (dt) (dt)^2 (dt)^3 (dt)^4 (dt)^5;
           0 1 2*dt^1 3*dt^2 4*(dt)^3 5*(dt)^4;
           0 0 2 6*dt 12*(dt)^2 20*(dt)^3;];
        m=inv(m);
        A(1+j*6+12:6+j*6+12,1+(j-1)/2*9+3+9:6+(j-1)/2*9+3+9)=m;%A(1+j*4+8:4+j*4+8,1+(j-1)/2*6+2+6:4+(j-1)/2*6+2+6)=m;
        j=j+1;
    elseif(type_phase(i)==1)
        m=[1 0 0 0 0 0;
           0 1 0 0 0 0;
           0 0 2 0 0 0;
           1 (dt) (dt)^2 (dt)^3 (dt)^4 (dt)^5;
           0 1 2*dt^1 3*dt^2 4*(dt)^3 5*(dt)^4;
           0 0 2 6*dt 12*(dt)^2 20*(dt)^3;];
      m=inv(m);
      A(1+j*6+12:6+j*6+12,1+j/2*9+9:6+j/2*9+9)=m;%A(1+j*4+8:4+j*4+8,1+j/2*6+6:4+j/2*6+6)=m;
      j=j+1;
    end

end

dt=tpassage(end-1)-tpassage(end-2);
dt=dt*0.5;
m1=[1 0 0 0 0 0;
   0 1 0 0 0 0;
   0 0 2 0 0 0;
   1 (dt) (dt)^2 (dt)^3 (dt)^4 (dt)^5;
   0 1 2*dt^1 3*dt^2 4*(dt)^3 5*(dt)^4;
   0 0 2 6*dt 12*(dt)^2 20*(dt)^3;];
m1=inv(m1);
m2=[1 0 0 0 0 0;
   0 1 0 0 0 0;
   0 0 2 0 0 0;
   1 (dt) (dt)^2 (dt)^3 (dt)^4 (dt)^5;
   0 1 2*dt^1 3*dt^2 4*(dt)^3 5*(dt)^4;
   0 0 2 6*dt 12*(dt)^2 20*(dt)^3;];
m2=inv(m2);
m=[m1 zeros(6,3);
   zeros(6,3) m2;];
A(end-11:end,end-8:end)=m;

end