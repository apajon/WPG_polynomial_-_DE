function [Apankle Bpankle] = torque_ankle_positions_SSP2(pankinit,pankfin,discretization)
nbphases=length(discretization);
Apankle=zeros(1+sum(discretization),ceil(nbphases/3));
%pankle(1)=0;
nb=0;
rowinitphase=1;
for j=1:nbphases
    rowinitphase=rowinitphase+nb;
    nb=discretization(j);
    if(mod(j,3)==0 || j==1 || j==nbphases || j==2 || j==nbphases-1)
%         for i=1:nb
%             pankle=[pankle;0];
%         end
    else
        for i=1:nb
        Apankle(rowinitphase+i,ceil(j/3))=1;
        end
    end
end

Apankle=Apankle(:,any(Apankle,1));

Bpankle=zeros(1+sum(discretization),1);
Bpankle(1+discretization(1)+1:1+sum(discretization(1:2)))=pankinit*ones(discretization(2),1);
Bpankle(1+sum(discretization(1:end-2))+1:1+sum(discretization(1:end-1)))=pankfin*ones(discretization(end-1),1);

end