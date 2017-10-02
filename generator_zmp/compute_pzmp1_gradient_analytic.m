%Compute the gradient matrix of zmp position
function [A B]=compute_pzmp1_gradient_analytic(Apzmp,Bpzmp,Apankle1,Bpankle1,Apankle2,Bpankle2,Afb,mg)
%xf2=(eye(length(Afb))-Afb)*mg;
% ixf2=inv(xf2);

%A=-mg*(xf2\(Apzmp1-[Apzmp zeros(size(Apzmp,1),size(Apzmp1,2)-size(Apzmp,2))]))+Apzmp1;
%B=-mg*(xf2\(Bpzmp1-Bpzmp))+Bpzmp1;

A=mg*((2*Afb*mg)\([Apzmp -Apankle2]))+([zeros(size(Apzmp)) Apankle1+Apankle2])/2;
B=mg*((2*Afb*mg)\(Bpzmp-Bpankle2))+(Bpankle1+Bpankle2)/2;
end