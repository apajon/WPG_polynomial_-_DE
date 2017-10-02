function [mscwt] = compute_coeff_dswt_dcwt_matrix(discretization,w,frequency)
nbphases=length(discretization);
mscwt= zeros(1+sum(discretization),(nbphases)*2);
mscwt(1,1:2)=[w*sinh(0) w*cosh(0)];
nb=0;
rowinitphase=1;
for j=1:nbphases
    rowinitphase=rowinitphase+nb;
    nb=discretization(j);
    for i=1:nb
        dt=i/frequency;%dt is delta(tj)=t-Tj
        mscwt(rowinitphase+i,(j-1)*2+1:(j-1)*2+2)=[w*sinh(w*dt) w*cosh(w*dt)];
    end
end
end