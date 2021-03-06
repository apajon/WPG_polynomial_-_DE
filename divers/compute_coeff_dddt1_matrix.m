function [mdt] = compute_coeff_dddt1_matrix(discretization,frequency,cutting)
nbphases=length(discretization);
nbpoints=sum(discretization)+1;
mdt=zeros(nbpoints,nbphases*6+6+6);

mdt(1,1:6)=[0 0 2 0 0 0];

%%%zmp1 init cut in two%%%
nb=discretization(1);
rowinitphase=1;
for i=1:ceil(nb*cutting)
    dt=i/frequency;%dt is delta(tj)=t-Tj
    Dt=[0 0 2 6*dt^1 12*dt^2 20*dt^3];
    mdt(rowinitphase+i,(1-1)*6+1:(1-1)*6+6)=Dt;
end
rowinitphase=1+ceil(nb*cutting);
for i=1:nb-ceil(nb*cutting)
    dt=i/frequency;%dt is delta(tj)=t-Tj
    Dt=[0 0 2 6*dt^1 12*dt^2 20*dt^3];
    mdt(rowinitphase+i,(2-1)*6+1:(2-1)*6+6)=Dt;
end

rowinitphase=1;
%%%generation of dt%%%
for j=2:nbphases-1
    rowinitphase=rowinitphase+nb;
    nb=discretization(j);
    for i=1:nb
        dt=i/frequency;%dt is delta(tj)=t-Tj
        Dt=[0 0 2 6*dt^1 12*dt^2 20*dt^3];
            mdt(rowinitphase+i,(j+1-1)*6+1:(j+1-1)*6+6)=Dt;
    end
end

%%%zmp1 fin cut in two%%%
rowinitphase=rowinitphase+nb;
nb=discretization(end);
for i=1:ceil(nb*cutting)
    dt=i/frequency;%dt is delta(tj)=t-Tj
    Dt=[0 0 2 6*dt^1 12*dt^2 20*dt^3];
    mdt(rowinitphase+i,(nbphases+1-1)*6+1:(nbphases+1-1)*6+6)=Dt;
end
rowinitphase=rowinitphase+ceil(nb*cutting);
for i=1:nb-ceil(nb*cutting)
    dt=i/frequency;%dt is delta(tj)=t-Tj
    Dt=[0 0 2 6*dt^1 12*dt^2 20*dt^3];
    mdt(rowinitphase+i,(nbphases+2-1)*6+1:(nbphases+2-1)*6+6)=Dt;
end

end