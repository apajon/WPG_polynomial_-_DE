function [A B]=com_morisawa_VW_gradient_global(A_gradient,tpassage,w)
%put a system Z1*y=X*x+Z2*L as y=A*a+B*x
%y is a vector of V and W scalar coeff of COM trajectory
%x is a vector with the initial and final known COM position
%a is a vector build of ZMP polynomial coeff
%A=compute_A(a,length(tpassage)+1,w);
% A_gradient=com_morisawa_A_gradient(length(tpassage)-1,w);
Z1=compute_Z1(tpassage,w);
Z2=compute_Z2(tpassage);

X=[1 0 0 0; zeros(size(Z2,1)-2,4) ;0 1 0 0]; %pcominit pconfin scominit scomfin

A=Z1\(Z2*A_gradient);
B=Z1\(X);

end

% %Compute the matrix gradient of A vector
% function [A]=compute_A_gradient(nbphases,w)
% A=[];
% for j=1:nbphases
%     Aj=[1 0 2/w^2 0 24/w^4 0;
%         0 1 0 6/w^2 0 120/w^4;
%         0 0 1 0 12/w^2 0;
%         0 0 0 1 0 30/w^2;
%         0 0 0 0 1 0;
%         0 0 0 0 0 1];
%     A=[A;zeros(6,(j-1)*6) Aj zeros(6,(nbphases-j)*6)];
% end
% end

%Compute the Z1 matrix
function [Z1]=compute_Z1(tpassage,w)
nbphases=length(tpassage)-1;

Z1=[1 0 zeros(1,2*nbphases-2)];

for i=1:nbphases-1
    Dt=tpassage(i+1)-tpassage(i);
    Z1=[Z1;zeros(1,2*(i-1)) cosh(w*Dt) sinh(w*Dt) -1 0 zeros(1,2*(nbphases-i)-2);zeros(1,2*(i-1)) w*sinh(w*Dt) w*cosh(w*Dt) 0 -w zeros(1,2*(nbphases-i)-2)];
end

Dt=tpassage(nbphases+1)-tpassage(nbphases);
Z1=[Z1;zeros(1,2*nbphases-2) cosh(w*Dt) sinh(w*Dt)];
    
end

%compute the Z2 matrix
function [Z2]=compute_Z2(tpassage)
nbphases=length(tpassage)-1;

Z2=[-1 0 zeros(1,6*nbphases-2)];

for i=1:nbphases-1
    Dt=tpassage(i+1)-tpassage(i);
    Z2=[Z2;zeros(1,6*(i-1)) -Dt^0 -Dt^1 -Dt^2 -Dt^3 -Dt^4 -Dt^5 +1 0 zeros(1,6*(nbphases-i)-2);zeros(1,6*(i-1)) 0 -1*Dt^0 -2*Dt^1 -3*Dt^2 -4*Dt^3 -5*Dt^4 0 +1 zeros(1,6*(nbphases-i)-2)];
end

Dt=tpassage(nbphases+1)-tpassage(nbphases);
Z2=[Z2;zeros(1,6*(nbphases-1)) -Dt^0 -Dt^1 -Dt^2 -Dt^3 -Dt^4 -Dt^5];

end