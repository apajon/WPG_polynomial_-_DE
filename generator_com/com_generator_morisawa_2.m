%%%generate the COM trajectory during 1 walking sequence%%%
%Use Morisawa algorithm with zmp polynomial coeff known
function [com time] = com_generator_morisawa_2(psa_abcd,Apcom,Bpcom,tpassage,frequency) %rajouter le calcul de scom
% [Apcom Bpcom] = pcom_generator_morisawa_gradient(tpassage,psa_zmpinit,psa_zmpfin,pcominit,pcomfin,w,frequency);

com=Apcom*psa_abcd+Bpcom;

time=[];
for i=0:round(tpassage(length(tpassage))*frequency)
    time=[time;i/frequency];
end
end
