function dt_type_phase=compute_dt_type_phase(type_phase,discretization,nbphases,nbpointdiscret)

dt_type_phase=zeros(nbpointdiscret,size(type_phase,1));
dt_type_phase(1,:)=type_phase(:,1)';
nb=0;
rowinitphase=1;
for j=1:nbphases
    rowinitphase=rowinitphase+nb;
    nb=discretization(j);
    for i=1:nb
           dt_type_phase(rowinitphase+i,:)=type_phase(:,j)';
    end
end