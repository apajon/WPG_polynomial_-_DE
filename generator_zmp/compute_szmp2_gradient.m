%Compute the gradient matrix of zmp speed
function [A B]=compute_szmp2_gradient(Apzmp,Bpzmp,Apzmp1,Bpzmp1,Afb,nbparamank,nbparamBb,Aszmp,Bszmp,Aszmp1,Bszmp1,dAfb)
Afb_=(eye(length(Afb))-Afb);

Apzmp_=[Apzmp zeros(size(Apzmp,1),nbparamank+nbparamBb)];

Aszmp_=[Aszmp zeros(size(Aszmp,1),nbparamank+nbparamBb)];

A=-(Afb_\(Aszmp1-Aszmp_))+dAfb*((Afb_)\((Afb_)\(Apzmp1-Apzmp_)))+Aszmp1;
B=-(Afb_\(Bszmp1-Bszmp))+dAfb*((Afb_)\((Afb_)\(Bpzmp1-Bpzmp)))+Bszmp1;
end