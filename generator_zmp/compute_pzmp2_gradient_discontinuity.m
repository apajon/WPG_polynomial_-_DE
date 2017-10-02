%Compute the gradient matrix of zmp position
function [A B]=compute_pzmp2_gradient_discontinuity(Apzmp,Bpzmp,Apzmp1,Bpzmp1,Afb,mg,nbparamank)
xf2=(eye(length(Afb))-Afb)*mg;
% ixf2=inv(xf2);

Apzmp1_=[zeros(size(Apzmp)) zeros(size(Apzmp1,1),nbparamank) Apzmp1];
Apzmp_=[Apzmp zeros(size(Apzmp,1),nbparamank) zeros(size(Apzmp1))];

A=-mg*(xf2\(Apzmp1_-Apzmp_))+Apzmp1_;
B=-mg*(xf2\(Bpzmp1-Bpzmp))+Bpzmp1;
end