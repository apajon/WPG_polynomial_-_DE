function [mdt] = compute_coeff_ddt_pankle_matrix(discretization,frequency,nbphases,nbpointdiscret,type_phase)

cutting=0.5;

mdt=zeros(nbpointdiscret,nbphases*6+3+3);

%%%zmp1 init cut in two%%%
nb=discretization(2);
rowinitphase=1+discretization(1);
for i=1:ceil(nb*cutting)
    dt=i/frequency;%dt is delta(tj)=t-Tj
    Dt=[0 1 2*dt^1 3*dt^2 4*dt^3 5*dt^4];
    mdt(rowinitphase+i,(1-1)*6+1:(1-1)*6+6)=Dt;
end
rowinitphase=rowinitphase+ceil(nb*cutting);
for i=1:nb-ceil(nb*cutting)
    dt=i/frequency;%dt is delta(tj)=t-Tj
    Dt=[0 1 2*dt^1 3*dt^2 4*dt^3 5*dt^4];
    mdt(rowinitphase+i,(2-1)*6+1:(2-1)*6+6)=Dt;
end

rowinitphase=1+discretization(1);
%%%generation of dt%%%
for j=3:nbphases-2
    rowinitphase=rowinitphase+nb;
    nb=discretization(j);
    if(type_phase(j)~=0)
        for i=1:nb
            dt=i/frequency;%dt is delta(tj)=t-Tj
            Dt=[0 1 2*dt^1 3*dt^2 4*dt^3 5*dt^4];
                mdt(rowinitphase+i,(j+1-1)*6+1:(j+1-1)*6+6)=Dt;
        end
    end
end

%%%zmp1 fin cut in two%%%
rowinitphase=rowinitphase+nb;
nb=discretization(end-1);
for i=1:ceil(nb*cutting)
    dt=i/frequency;%dt is delta(tj)=t-Tj
    Dt=[0 1 2*dt^1 3*dt^2 4*dt^3 5*dt^4];
    mdt(rowinitphase+i,end-11:end-6)=Dt;
end
rowinitphase=rowinitphase+ceil(nb*cutting);
for i=1:nb-ceil(nb*cutting)
    dt=i/frequency;%dt is delta(tj)=t-Tj
    Dt=[0 1 2*dt^1 3*dt^2 4*dt^3 5*dt^4];
    mdt(rowinitphase+i,end-5:end)=Dt;
end

end