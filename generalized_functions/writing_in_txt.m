function  []=writing_in_txt(walking_param,save_pstep,save_trajectories)
load('generalized_functions/support_foot.mat');

%%
trajectories=[dt_type_phase_ ...
        walking_param.xpzmp-0.0095 walking_param.ypzmp ones(size(walking_param.xpzmp))*0 ...
        walking_param.xpcom-0.0095 walking_param.ypcom ones(size(walking_param.xpcom))*walking_param.z ...
        walking_param.xscom walking_param.yscom zeros(size(walking_param.xscom)) ...
        walking_param.xacom walking_param.yacom zeros(size(walking_param.xacom)) ...
        walking_param.xpankle_l-0.0095 walking_param.ypankle_l walking_param.zpankle_l ...%-any(zpinair_r,2)*ha ...
        walking_param.xsankle_l walking_param.ysankle_l walking_param.zsankle_l ...
        walking_param.xaankle_l walking_param.yaankle_l walking_param.zaankle_l ...
        walking_param.xpankle_r-0.0095 walking_param.ypankle_r walking_param.zpankle_r ...%-any(zpinair_r,2)*ha ...
        walking_param.xsankle_r walking_param.ysankle_r walking_param.zsankle_r ...
        walking_param.xaankle_r walking_param.yaankle_r walking_param.zaankle_r ...
        walking_param.rtheta_l walking_param.rphi_l walking_param.rpsi_l ... %rtheta_dt_r rphi_dt_r
        walking_param.rtheta_r walking_param.rphi_r walking_param.rpsi_r]; %rtheta_dt_r rphi_dt_r
%%%%%%%%%%%

%% %save data in txt
zmpcom=fopen(save_trajectories,'w');
for i=1:size(trajectories,1)
    fprintf(zmpcom,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n',trajectories(i,:));
end
fclose(zmpcom);
%% %save data in txt
% type_phase=fopen('type_phase.txt','w');
% pzmp=fopen('pzmp.txt','w');
% pcom=fopen('pcom.txt','w');
% scom=fopen('scom.txt','w');
% acom=fopen('acom.txt','w');
% pinair_l=fopen('pinair_l.txt','w');
% pinair_r=fopen('pinair_r.txt','w');
% vinair_l=fopen('vinair_l.txt','w');
% vinair_r=fopen('vinair_r.txt','w');
% ainair_l=fopen('ainair_l.txt','w');
% ainair_r=fopen('ainair_r.txt','w');
% ponfloor_l=fopen('ponfloor_l.txt','w');
% ponfloor_r=fopen('ponfloor_r.txt','w');
% vonfloor_l=fopen('vonfloor_l.txt','w');
% vonfloor_r=fopen('vonfloor_r.txt','w');
% aonfloor_l=fopen('aonfloor_l.txt','w');
% aonfloor_r=fopen('aonfloor_r.txt','w');
% angle=fopen('angle.txt','w');
% for i=1:size(trajectories,1)
%     fprintf(type_phase,'%f\n',dt_type_phase(i,:));
%     fprintf(pzmp,'%f %f %f\n',[xpzmp-0.0190 ypzmp ones(size(xpzmp))*0]);
%     fprintf(pcom,'%f %f %f\n',[xpcom-0.0190 ypcom ones(size(xpcom))*z]);
%     fprintf(scom,'%f %f %f\n',[xscom yscom zeros(size(xscom))]);
%     fprintf(acom,'%f %f %f\n',[xacom yacom zeros(size(xacom))]);
%     fprintf(pinair_l,'%f %f %f\n',[xpinair_l-any(xpinair_l,2)*0.0095 ypinair_l zpinair_l-any(zpinair_l,2)*ha]);
%     fprintf(pinair_r,'%f %f %f\n',[xpinair_r-any(xpinair_r,2)*0.0095 ypinair_r zpinair_r-any(zpinair_l,2)*ha]);
%     fprintf(vinair_l,'%f %f %f\n',[xvinair_l yvinair_l zvinair_l]);
%     fprintf(vinair_r,'%f %f %f\n',[xvinair_r yvinair_r zvinair_r]);
%     fprintf(ainair_l,'%f %f %f\n',[xainair_l yainair_l zainair_l]);
%     fprintf(ainair_r,'%f %f %f\n',[xainair_r yainair_r zainair_r]);
%     fprintf(ponfloor_l,'%f %f %f\n',[xponfloor_l yponfloor_l zponfloor_l]);
%     fprintf(ponfloor_r,'%f %f %f\n',[xponfloor_r yponfloor_r zponfloor_r]);
%     fprintf(vonfloor_l,'%f %f %f\n',[xvonfloor_l yvonfloor_l zvonfloor_l]);
%     fprintf(vonfloor_r,'%f %f %f\n',[xvonfloor_r yvonfloor_r zvonfloor_r]);
%     fprintf(aonfloor_l,'%f %f %f\n',[xaonfloor_l yaonfloor_l zaonfloor_l]);
%     fprintf(aonfloor_r,'%f %f %f\n',[xaonfloor_r yaonfloor_r zaonfloor_r]);
%     fprintf(angle,'%f %f %f\n',[rpsi_dt zeros(size(rpsi_dt)) zeros(size(rpsi_dt))]);
% end
% fclose('all');

%%%%%%%%%%%

%%
psi_=walking_param.psi((1:walking_param.nbstep)*3);
psi_=[walking_param.psi(1) psi_ walking_param.psi(end-1:end)]';

pstep_=[walking_param.pstep(:,1)-0.0095 walking_param.pstep(:,2) psi_];
pstepf=fopen(save_pstep,'w');
for i=1:size(pstep_,1)
    fprintf(pstepf,'%f %f %f\n',pstep_(i,:));
end
fclose(pstepf);

fclose('all');
%%%%%%%%%%%
end