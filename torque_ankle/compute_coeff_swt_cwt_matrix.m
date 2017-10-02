function [mscwt] = compute_coeff_swt_cwt_matrix(discretization,w,frequency,nbphases)

mscwt= zeros(1+sum(discretization),(nbphases)*2);
mscwt(1,1:2)=[cosh(0) sinh(0)];
nb=0;
rowinitphase=1;
for j=1:nbphases
    rowinitphase=rowinitphase+nb;
    nb=discretization(j);
    for i=1:nb
        dt=i/frequency;%dt is delta(tj)=t-Tj
        mscwt(rowinitphase+i,(j-1)*2+1:(j-1)*2+2)=[cosh(w*dt) sinh(w*dt)];
    end
end
end