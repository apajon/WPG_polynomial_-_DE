function [time_ pzmp pcom fzmp] = zmp_under_foot(foot_step_wanted,nbpankle,time,xpzmp,ypzmp,xpzmp1,ypzmp1,xpzmp2,ypzmp2,xpcom,ypcom,zpcom,zfzmp1,xAfcom,xBfcom,yAfcom,yBfcom,psa_abcd,discretization,discretization_,mg)
%% % adapt zmp and ZMP1&2 to have the CoP under the wanted foot step
if foot_step_wanted == 1
    xpzmp1_=xpzmp1(1:sum(discretization_(1))+1);
    ypzmp1_=ypzmp1(1:sum(discretization_(1))+1);
    
    xpzmp_=[xpzmp1_];
    ypzmp_=[ypzmp1_];

    time_=time(1:sum(discretization(1))+1);

    xpcom_=xpcom(1:sum(discretization(1))+1);
    ypcom_=ypcom(1:sum(discretization(1))+1);
    zpcom_=zpcom(1:sum(discretization(1))+1);

    zfzmp1_=zfzmp1(1:sum(discretization_(1))+1)*mg;
    zfzmp_=[zfzmp1_;];

    xfzmp=-(xAfcom*psa_abcd(1:length(psa_abcd)/2)+xBfcom);
    yfzmp=-(yAfcom*psa_abcd(length(psa_abcd)/2+1:end)+yBfcom);
    xfzmp_=xfzmp(1:sum(discretization(1))+1).*(zfzmp_/mg);
    yfzmp_=yfzmp(1:sum(discretization(1))+1).*(zfzmp_/mg);
elseif foot_step_wanted == 2
    xpzmp1_=xpzmp1(sum(discretization_(1:foot_step_wanted-1))+1+1:sum(discretization_(1:foot_step_wanted))+1);
    ypzmp1_=ypzmp1(sum(discretization_(1:foot_step_wanted-1))+1+1:sum(discretization_(1:foot_step_wanted))+1);
    xpzmp2_=xpzmp2(1:sum(discretization_(1:foot_step_wanted-1))+1);
    ypzmp2_=ypzmp2(1:sum(discretization_(1:foot_step_wanted-1))+1);
    xpzmp_=xpzmp(sum(discretization(foot_step_wanted-1))+1+1:sum(discretization(1:foot_step_wanted))+1);
    ypzmp_=ypzmp(sum(discretization(foot_step_wanted-1))+1+1:sum(discretization(1:foot_step_wanted))+1);

    xpzmp_=[xpzmp2_;xpzmp_;xpzmp1_];
    ypzmp_=[ypzmp2_;ypzmp_;ypzmp1_];

    time_=time(1:sum(discretization(1:3))+1);

    xpcom_=xpcom(1:sum(discretization(1:(foot_step_wanted-3)*3+3+3))+1);
    ypcom_=ypcom(1:sum(discretization(1:(foot_step_wanted-3)*3+3+3))+1);
    zpcom_=zpcom(1:sum(discretization(1:(foot_step_wanted-3)*3+3+3))+1);

    zfzmp1_=zfzmp1(sum(discretization_(1:foot_step_wanted-1))+1+1:sum(discretization_(1:foot_step_wanted))+1)*mg;
    zfzmp2_=(1-zfzmp1(1:sum(discretization_(1))+1))*mg;
    zfzmp_=ones(discretization(2),1)*mg;
    zfzmp_=[zfzmp2_;zfzmp_;zfzmp1_;];

    xfzmp=-(xAfcom*psa_abcd(1:length(psa_abcd)/2)+xBfcom);
    yfzmp=-(yAfcom*psa_abcd(length(psa_abcd)/2+1:end)+yBfcom);
    xfzmp_=xfzmp(1:sum(discretization(1:(foot_step_wanted-3)*3+3+3))+1).*(zfzmp_/mg);
    yfzmp_=yfzmp(1:sum(discretization(1:(foot_step_wanted-3)*3+3+3))+1).*(zfzmp_/mg);
elseif foot_step_wanted==nbpankle-1
    xpzmp1_=xpzmp1(sum(discretization_(1:foot_step_wanted-1))+1+1:sum(discretization_(1:foot_step_wanted))+1);
    ypzmp1_=ypzmp1(sum(discretization_(1:foot_step_wanted-1))+1+1:sum(discretization_(1:foot_step_wanted))+1);
    xpzmp2_=xpzmp2(sum(discretization_(1:foot_step_wanted-2))+1+1:sum(discretization_(1:foot_step_wanted-1))+1);
    ypzmp2_=ypzmp2(sum(discretization_(1:foot_step_wanted-2))+1+1:sum(discretization_(1:foot_step_wanted-1))+1);
    xpzmp_=xpzmp(sum(discretization(1:(foot_step_wanted-3)*3+3))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+1))+1);
    ypzmp_=ypzmp(sum(discretization(1:(foot_step_wanted-3)*3+3))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+1))+1);

    xpzmp_=[xpzmp2_;xpzmp_;xpzmp1_];
    ypzmp_=[ypzmp2_;ypzmp_;ypzmp1_];

    time_=time(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+2))+1);

    xpcom_=xpcom(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+2))+1);
    ypcom_=ypcom(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+2))+1);
    zpcom_=zpcom(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+2))+1);

    zfzmp1_=zfzmp1(sum(discretization_(1:foot_step_wanted-1))+1+1:sum(discretization_(1:foot_step_wanted))+1)*mg;
    zfzmp2_=(1-zfzmp1(sum(discretization_(1:foot_step_wanted-2))+1+1:sum(discretization_(1:foot_step_wanted-1))+1))*mg;
    zfzmp_=ones(discretization(end-1),1)*mg;
    zfzmp_=[zfzmp2_;zfzmp_;zfzmp1_;];

    xfzmp=-(xAfcom*psa_abcd(1:length(psa_abcd)/2)+xBfcom);
    yfzmp=-(yAfcom*psa_abcd(length(psa_abcd)/2+1:end)+yBfcom);
    xfzmp_=xfzmp(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+2))+1).*(zfzmp_/mg);
    yfzmp_=yfzmp(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+2))+1).*(zfzmp_/mg);
elseif foot_step_wanted==nbpankle
    xpzmp2_=xpzmp2(sum(discretization_(1:foot_step_wanted-2))+1+1:sum(discretization_(1:foot_step_wanted-1))+1);
    ypzmp2_=ypzmp2(sum(discretization_(1:foot_step_wanted-2))+1+1:sum(discretization_(1:foot_step_wanted-1))+1);

    xpzmp_=[xpzmp2_;];
    ypzmp_=[ypzmp2_;];

    time_=time(sum(discretization(1:(foot_step_wanted-3)*3+1))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+2))+1);

    xpcom_=xpcom(sum(discretization(1:(foot_step_wanted-3)*3+1))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+2))+1);
    ypcom_=ypcom(sum(discretization(1:(foot_step_wanted-3)*3+1))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+2))+1);
    zpcom_=zpcom(sum(discretization(1:(foot_step_wanted-3)*3+1))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+2))+1);

    zfzmp2_=(1-zfzmp1(sum(discretization_(1:foot_step_wanted-2))+1+1:sum(discretization_(1:foot_step_wanted-1))+1))*mg;
    zfzmp_=[zfzmp2_;];

    xfzmp=-(xAfcom*psa_abcd(1:length(psa_abcd)/2)+xBfcom);
    yfzmp=-(yAfcom*psa_abcd(length(psa_abcd)/2+1:end)+yBfcom);
    xfzmp_=xfzmp(sum(discretization(1:(foot_step_wanted-3)*3+1))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+2))+1).*(zfzmp_/mg);
    yfzmp_=yfzmp(sum(discretization(1:(foot_step_wanted-3)*3+1))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+2))+1).*(zfzmp_/mg);
else
    xpzmp1_=xpzmp1(sum(discretization_(1:foot_step_wanted-1))+1+1:sum(discretization_(1:foot_step_wanted))+1);
    ypzmp1_=ypzmp1(sum(discretization_(1:foot_step_wanted-1))+1+1:sum(discretization_(1:foot_step_wanted))+1);
    xpzmp2_=xpzmp2(sum(discretization_(1:foot_step_wanted-2))+1+1:sum(discretization_(1:foot_step_wanted-1))+1);
    ypzmp2_=ypzmp2(sum(discretization_(1:foot_step_wanted-2))+1+1:sum(discretization_(1:foot_step_wanted-1))+1);
    xpzmp_=xpzmp(sum(discretization(1:(foot_step_wanted-3)*3+3))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+2))+1);
    ypzmp_=ypzmp(sum(discretization(1:(foot_step_wanted-3)*3+3))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+2))+1);

    xpzmp_=[xpzmp2_;xpzmp_;xpzmp1_];
    ypzmp_=[ypzmp2_;ypzmp_;ypzmp1_];

    time_=time(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+3))+1);

    xpcom_=xpcom(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+3))+1);
    ypcom_=ypcom(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+3))+1);
    zpcom_=zpcom(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+3))+1);

    zfzmp1_=zfzmp1(sum(discretization_(1:foot_step_wanted-1))+1+1:sum(discretization_(1:foot_step_wanted))+1)*mg;
    zfzmp2_=(1-zfzmp1(sum(discretization_(1:foot_step_wanted-2))+1+1:sum(discretization_(1:foot_step_wanted-1))+1))*mg;
    zfzmp_=ones(sum(discretization((foot_step_wanted-3)*3+4:(foot_step_wanted-3)*3+3+2)),1)*mg;
    zfzmp_=[zfzmp2_;zfzmp_;zfzmp1_;];

    xfzmp=-(xAfcom*psa_abcd(1:length(psa_abcd)/2)+xBfcom);
    yfzmp=-(yAfcom*psa_abcd(length(psa_abcd)/2+1:end)+yBfcom);
    xfzmp_=xfzmp(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+3))+1).*(zfzmp_/mg);
    yfzmp_=yfzmp(sum(discretization(1:(foot_step_wanted-3)*3+2))+1+1:sum(discretization(1:(foot_step_wanted-3)*3+3+3))+1).*(zfzmp_/mg);
end

pzmp=[xpzmp_ ypzmp_];
pcom=[xpcom_ ypcom_ zpcom_];
fzmp=[xfzmp_ yfzmp_ zfzmp_];

% %%
% trajectories=[time_' xpzmp_ ypzmp_ xpcom_ ypcom_ xfzmp yfzmp zfzmp];
% 
% zmpcom=fopen('exemple_trajectoire.txt','w');
% for i=1:size(trajectories,1)
%     fprintf(zmpcom,'%f %f %f %f %f %f %f %f\n',trajectories(i,:));
% end
% fclose(zmpcom);
% %%%%%%%%%%%
end