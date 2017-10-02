clc

%% %%%speed of com computation
% [xAscom xBscom]=scom_generator_morisawa_gradient(xAzmp_gradient,xBzmp_gradient,xAVW,xBVW,A_gradient,discretization,w,frequency);
% xscom=xAscom*psa_abcd(1:length(psa_abcd)/2)+xBscom;
% [yAscom yBscom]=scom_generator_morisawa_gradient(yAzmp_gradient,yBzmp_gradient,yAVW,yBVW,A_gradient,discretization,w,frequency);
% yscom=yAscom*psa_abcd(length(psa_abcd)/2+1:length(psa_abcd))+yBscom;
[xAscom xBscom]=scom_generator_morisawa_gradient(xAzmp_gradient,xBzmp_gradient,xAVW,xBVW,A_gradient,discretization,w,frequency,ddt);
xscom=xAscom*psa_abcd(1:length(psa_abcd)/2)+xBscom;
[yAscom yBscom]=scom_generator_morisawa_gradient(yAzmp_gradient,yBzmp_gradient,yAVW,yBVW,A_gradient,discretization,w,frequency,ddt);
yscom=yAscom*psa_abcd(length(psa_abcd)/2+1:length(psa_abcd))+yBscom;

%% %%%acceleration of com computation
xacom=(xpcom-xpzmp)*g/z;
yacom=(ypcom-ypzmp)*g/z;

%% %recompute dt_type_phase
dt_type_phase_=any(zvinair_l,2)*2+any(zvinair_r,2)*1;

%% %%%writing ZMP and COM trajectories in 'zmp&com.txt' file%%%%
xpankle_l=xpinair_l+xponfloor_l;
ypankle_l=ypinair_l+yponfloor_l;
zpankle_l=zpinair_l+zponfloor_l;
xpankle_r=xpinair_r+xponfloor_r;
ypankle_r=ypinair_r+yponfloor_r;
zpankle_r=zpinair_r+zponfloor_r;

xsankle_l=([0;(xpankle_l(2:end)-xpankle_l(1:end-1))*frequency;]);
ysankle_l=([0;(ypankle_l(2:end)-ypankle_l(1:end-1))*frequency;]);
zsankle_l=([0;(zpankle_l(2:end)-zpankle_l(1:end-1))*frequency;]);
xsankle_r=([0;(xpankle_r(2:end)-xpankle_r(1:end-1))*frequency;]);
ysankle_r=([0;(ypankle_r(2:end)-ypankle_r(1:end-1))*frequency;]);
zsankle_r=([0;(zpankle_r(2:end)-zpankle_r(1:end-1))*frequency;]);

xaankle_l=([0;(xpankle_l(3:end)+xpankle_l(1:end-2)-2*xpankle_l(2:end-1))*frequency;0]);
yaankle_l=([0;(ypankle_l(3:end)+ypankle_l(1:end-2)-2*ypankle_l(2:end-1))*frequency;0]);
zaankle_l=([0;(zpankle_l(3:end)+zpankle_l(1:end-2)-2*zpankle_l(2:end-1))*frequency;0]);
xaankle_r=([0;(xpankle_r(3:end)+xpankle_r(1:end-2)-2*xpankle_r(2:end-1))*frequency;0]);
yaankle_r=([0;(ypankle_r(3:end)+ypankle_r(1:end-2)-2*ypankle_r(2:end-1))*frequency;0]);
zaankle_r=([0;(zpankle_r(3:end)+zpankle_r(1:end-2)-2*zpankle_r(2:end-1))*frequency;0]);

rtheta_l=rtheta_dt_l+rtheta_dt_l_;
rphi_l=rphi_dt_l+rphi_dt_l_;
rpsi_l=rpsi_dt_l+rpsi_dt_l_;
rtheta_r=rtheta_dt_r+rtheta_dt_r_;
rphi_r=rphi_dt_r+rphi_dt_r_;
rpsi_r=rpsi_dt_r+rpsi_dt_r_;
%%
trajectories=[dt_type_phase_ ...
        xpzmp-0.0095 ypzmp ones(size(xpzmp))*0 ...
        xpcom-0.0095 ypcom ones(size(xpcom))*(z+e_-e) ...
        xscom yscom zeros(size(xscom)) ...
        xacom yacom zeros(size(xacom)) ...
        xpankle_l-0.0095 ypankle_l zpankle_l+e_-e ...%-any(zpinair_r,2)*ha ...
        xsankle_l ysankle_l zsankle_l ...
        xaankle_l yaankle_l zaankle_l ...
        xpankle_r-0.0095 ypankle_r zpankle_r+e_-e ...%-any(zpinair_r,2)*ha ...
        xsankle_r ysankle_r zsankle_r ...
        xaankle_r yaankle_r zaankle_r ...
        rtheta_l rphi_l rpsi_l ... %rtheta_dt_r rphi_dt_r
        rtheta_r rphi_r rpsi_r]; %rtheta_dt_r rphi_dt_r
if type_traj==9
    trajectories=[dt_type_phase_ ...
        xpzmp-0.0095 ypzmp ones(size(xpzmp))*0 ...
        xpcom-0.0095 ypcom ones(size(xpcom))*(z+e_-e) ...
        xscom yscom zeros(size(xscom)) ...
        xacom yacom zeros(size(xacom)) ...
        xpankle_l*0 ypankle_l*0+0.0816 zpankle_l*0+0.093 ...%-any(zpinair_r,2)*ha ...
        xsankle_l*0 ysankle_l*0 zsankle_l*0 ...
        xaankle_l*0 yaankle_l*0 zaankle_l*0 ...
        xpankle_r*0 ypankle_r*0-0.0816 zpankle_r*0+0.093 ...%-any(zpinair_r,2)*ha ...
        xsankle_r*0 ysankle_r*0 zsankle_r*0 ...
        xaankle_r*0 yaankle_r*0 zaankle_r*0 ...
        rtheta_l*0 rphi_l*0 rpsi_l*0 ... %rtheta_dt_r rphi_dt_r
        rtheta_r*0 rphi_r*0 rpsi_r*0]; %rtheta_dt_r rphi_dt_r
    
    trajectories=[zeros(size(xpzmp,1),1) ...
        xpzmp-0.0095 ypzmp ones(size(xpzmp))*0 ...
        xpcom-0.0095 ypcom ones(size(xpcom))*(z+e_-e) ...
        xscom yscom zeros(size(xscom)) ...
        xacom yacom zeros(size(xacom)) ...
        zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1)+0.0816 zeros(size(xpzmp,1),1)+0.093 ...%-any(zpinair_r,2)*ha ...
        zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1) ...
        zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1) ...
        zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1)-0.0816 zeros(size(xpzmp,1),1)+0.093 ...%-any(zpinair_r,2)*ha ...
        zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1) ...
        zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1) ...
        zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1) ... %rtheta_dt_r rphi_dt_r
        zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1) zeros(size(xpzmp,1),1)]; %rtheta_dt_r rphi_dt_r
    
    
    trajectories=[dt_type_phase_*0 ...
        xpzmp-0.0095 ypzmp ones(size(xpzmp))*0 ...
        xpcom-0.0095 ypcom ones(size(xpcom))*(z) ...
        xscom yscom zeros(size(xscom)) ...
        xacom yacom zeros(size(xacom)) ...
        xpankle_l*0 ypankle_l*0+0.0816 (zpankle_l+e_-e)*0+0.093 ...
        xsankle_l*0 ysankle_l*0 zsankle_l*0 ...
        xaankle_l*0 yaankle_l*0 zaankle_l*0 ...
        xpankle_r*0 ypankle_r*0-0.0816 (zpankle_r+e_-e)*0+0.093 ...
        xsankle_r*0 ysankle_r*0 zsankle_r*0 ...
        xaankle_r*0 yaankle_r*0 zaankle_r*0 ...
        rtheta_l*0 rphi_l*0 rpsi_l*0 ... %rtheta_dt_r rphi_dt_r
        rtheta_r*0 rphi_r*0 rpsi_r*0]; %rtheta_dt_r rphi_dt_r
    
    trajectories=[dt_type_phase_ ...
        xpzmp-0.0095 ypzmp ones(size(xpzmp))*0 ...
        xpcom-0.0095 ypcom ones(size(xpcom))*(z) ...
        xscom yscom zeros(size(xscom)) ...
        xacom yacom zeros(size(xacom)) ...
        xpankle_l-0.0095+0 ypankle_l zpankle_l+e_-e ...
        xsankle_l*0 ysankle_l*0 zsankle_l*0 ...
        xaankle_l*0 yaankle_l*0 zaankle_l*0 ...
        xpankle_r-0.0095+0 ypankle_r zpankle_r+e_-e ...
        xsankle_r*0 ysankle_r*0 zsankle_r*0 ...
        xaankle_r*0 yaankle_r*0 zaankle_r*0 ...
        rtheta_l*0 rphi_l*0 rpsi_l*0 ... %rtheta_dt_r rphi_dt_r
        rtheta_r*0 rphi_r*0 rpsi_r*0]; %rtheta_dt_r rphi_dt_r
end
%%%%%%%%%%%

%% %save data in txt
zmpcom=fopen('zmp_com_9_100_step4_oscil16cm_tss1000_tds500_tpi2500.txt','w');

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
psi_=psi((1:nbstep)*3);
psi_=[psi(1) psi_ psi(end-1:end)]';

pstep_=[pstep(:,1)-0.0095 pstep(:,2) psi_];
pstepf=fopen('pstep_9_100_step4_oscil16cm_tss1000_tds500_tpi2500.txt','w');

for i=1:size(pstep_,1)
    fprintf(pstepf,'%f %f %f\n',pstep_(i,:));
end
fclose(pstepf);

fclose('all');
%%%%%%%%%%%