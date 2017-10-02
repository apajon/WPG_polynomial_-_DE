function [mscwt] = compute_coeff_ddswt_ddcwt_matrix(discretization,w,frequency)
nbphases=length(discretization);
mscwt= zeros(1+sum(discretization),(nbphases)*2);
mscwt(1,1:2)=[-w^2*cosh(0) w^2*sinh(0)];
nb=0;
rowinitphase=1;
for j=1:nbphases
    rowinitphase=rowinitphase+nb;
    nb=discretization(j);
    for i=1:nb
        dt=i/frequency;%dt is delta(tj)=t-Tj
        mscwt(rowinitphase+i,(j-1)*2+1:(j-1)*2+2)=[w^2*cosh(w*dt) w^2*sinh(w*dt)];
    end
end
end