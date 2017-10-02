%A penser à inverser les y et x
%le couple en x est fonction des y
%le couple en y est fonction des x
function [A B] = torque_ankle_SSP_gradient2(Apzmp,Bpzmp,Afcom,Bfcom,pankinit,pankfin,discretization,mg,ha)

% [Afcom Bfcom]=fcom_gradient(Apzmp,Bpzmp,Apcom,Bpcom,mg,h);
% [Apzmp Bpzmp]=compute_pzmp_gradient(tpassage,psa_zmpinit,psa_zmpfin,frequency);

[Apankle Bpankle]=torque_ankle_positions_SSP2(pankinit,pankfin,discretization);

A1=ha*Afcom+mg*Apzmp;
A2=-mg*Apankle;
A=[A1 zeros(size(A2,1),size(A2,2))]+[zeros(size(A1,1),size(A1,2)) A2];
B=ha*Bfcom+mg*Bpzmp-mg*Bpankle;

% nbphases=length(discretization);
% nbparameters=(nbphases-1)*3;

% A(1,:)=zeros(1,nbparameters);
% B(1)=0;
% nb=0;
% rowinitphase=1;
% for j=1:nbphases
%     rowinitphase=rowinitphase+nb;
%     nb=discretization(j);
%     if(mod(j,3)==0 || j==1 || j==nbphases)
%         A(rowinitphase+1:rowinitphase+nb,:)=zeros(nb,nbparameters);
%         B(rowinitphase+1:rowinitphase+nb)=zeros(nb,1);
%     end
% end

% xApzmpconstraint = xApzmp(any(ssp,2),:);
A=A(any(any(Apankle,2)+any(Bpankle,2),2),:);
B=B(any(any(Apankle,2)+any(Bpankle,2),2),:);

end