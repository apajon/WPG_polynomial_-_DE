%Compute the gradient matrix of zmp speed
function [A B]=compute_szmp2_gradient_discontinuity(Apzmp,Bpzmp,Apzmp1,Bpzmp1,Afb,nbparamank,Aszmp,Bszmp,Aszmp1,Bszmp1,dAfb)
xf2=(eye(length(Afb))-Afb);
% xf2=(eye(length(Afb))-Afb)*mg;
% ixf2=inv(xf2);

Apzmp1_=[zeros(size(Apzmp)) zeros(size(Apzmp1,1),nbparamank) Apzmp1];
Apzmp_=[Apzmp zeros(size(Apzmp,1),nbparamank) zeros(size(Apzmp1))];

Aszmp1_=[zeros(size(Aszmp)) zeros(size(Aszmp1,1),nbparamank) Aszmp1];
Aszmp_=[Aszmp zeros(size(Aszmp,1),nbparamank) zeros(size(Aszmp1))];


A=-(xf2\(Aszmp1_-Aszmp_))-dAfb*((xf2^2)\(Apzmp1_-Apzmp_))+Aszmp1_;
B=-(xf2\(Bszmp1-Bszmp))-dAfb*((xf2^2)\(Bpzmp1-Bpzmp))+Bszmp1;
% A=-mg*(xf2\(Apzmp1_-Apzmp_))+Apzmp1_;
% B=-mg*(xf2\(Bpzmp1-Bpzmp))+Bpzmp1;
end