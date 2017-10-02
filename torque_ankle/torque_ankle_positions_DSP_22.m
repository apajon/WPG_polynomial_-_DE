function [Apankle Bpankle] = torque_ankle_positions_DSP_22(pankinit2,pankfin1,pankfin2,discretization)
nbphases=length(discretization);
Apankle=zeros(1+sum(discretization),ceil(nbphases/3));
nb=0;
rowinitphase=1;
for j=1:nbphases-3
    rowinitphase=rowinitphase+nb;
    nb=discretization(j);
    if(mod(j,3)==0&&j>=3)
        for i=1:nb
             Apankle(rowinitphase+i,ceil(j/3))=1;
        end
    else
%         for i=1:nb
%         pankle(rowinitphase+i)=pstep(ceil(j/3));
%         end
    end
end
Apankle=Apankle(:,any(Apankle,1));

Bpankle=zeros(1+sum(discretization),1);
Bpankle(1)=pankinit2;
Bpankle(1+1:1+discretization(1))=pankinit2*ones(discretization(1),1);
Bpankle(1+sum(discretization(1:end-3))+1:1+sum(discretization(1:end-2)))=pankfin1*ones(discretization(end-2),1);
Bpankle(1+sum(discretization(1:end-1))+1:1+sum(discretization(1:end)))=pankfin2*ones(discretization(end),1);
end