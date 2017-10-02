clc

% %for windows
% addpath .\cost_viapoint
% addpath .\generator_zmp
% addpath .\generator_com
% addpath .\f_com
% addpath .\torque_ankle
% addpath .\divers

%for linux
% addpath ./cost_viapoint
% addpath ./generator_zmp
% addpath ./generator_com
% addpath ./f_com
% addpath ./torque_ankle
% addpath ./divers

xpzmp=[]; ypzmp=[]; xpcom=[]; ypcom=[];

switch(zmp_type)
    case {0,4,5,6,7}
    case 1
nbparamtotal=nbparamABCD+nbparamank;
    case {2,3}
nbparamtotal=nbparamABCD+nbparamank+nbparamBb;
    case {8}
        nbparamABCD=(nbstep*3+2+1)*3-6;
        nbparamABCD=nbparamABCD+4;
        nbparamtotal=nbparamABCD+nbparamank+nbparamBb;
end
%%
psa_abcd=[psa_abcdDSP(1:nbparamABCD);psa_abcdDSP(nbparamtotal+1:nbparamtotal+nbparamABCD)];
%%
pstep=[psa_abcdDSP(nbparamABCD+1:nbparamABCD+nbparamank) psa_abcdDSP(nbparamtotal+nbparamABCD+1:nbparamtotal+nbparamABCD+nbparamank)];
%%
pstep=[pankinit1;
    pankinit2;
    pstep;
    pankfin1;
    pankfin2];%pstep(end,1) pstep(end,2)-(-1)^(nbstep)*0.095*2];
%%
% pabcd=[];
%  for i=0:length(psa_abcd)/2/3-1
%      pabcd=[pabcd;psa_abcd(i*3+1) psa_abcd(i*3+nbparamABCD+1)];
%  end
pabcd=[psa_abcd(1:3:nbparamABCD-4) psa_abcd(nbparamABCD+1:3:nbparamABCD+nbparamABCD-4)];
%%%%%%%%%%
%%
%%%Clear figure(3) to draw y(x) coordinates%%%
figure(8);
clf;
set(gca,'fontsize',14)
axis equal;
title('ZMP projected on the ground')
xlabel('x(m)')
ylabel('y(m)')
hold on;
plot(pstep(:,1),pstep(:,2),'+')
plot(pabcd(:,1),pabcd(:,2),'o');
hold off
%%%%%%%%%%%
   
%% %%%using to draw zmp and com with new ZMP generator%%%
if 1

% [xpzmp xtimezmp]=zmp_generator_2(psa_abcd(1:length(psa_abcd)/2),xApzmp,xBpzmp,tpassage,frequency);
% [ypzmp ytimezmp]=zmp_generator_2(psa_abcd(length(psa_abcd)/2+1:end),yApzmp,yBpzmp,tpassage,frequency);
xpzmp=xApzmp*psa_abcd(1:length(psa_abcd)/2)+xBpzmp;
ypzmp=yApzmp*psa_abcd(length(psa_abcd)/2+1:end)+yBpzmp;

switch(zmp_type)
    case {0,4,5,6,7,8}
%         [xpzmp1 xtimezmp1prim]=zmp_generator_2(psa_abcdDSP(1:nbparamtotal),xApzmp1,xBpzmp1,tpassage,frequency);
%         [ypzmp1 ytimezmp1prim]=zmp_generator_2(psa_abcdDSP(nbparamtotal+1:nbparamtotal+nbparamtotal),yApzmp1,yBpzmp1,tpassage,frequency);
        xpzmp1=xApzmp1*psa_abcdDSP(1:nbparamtotal)+xBpzmp1;
        ypzmp1=yApzmp1*psa_abcdDSP(nbparamtotal+1:nbparamtotal+nbparamtotal)+yBpzmp1;
    case 1
        [xpzmp1 xtimezmp1prim]=zmp_generator_2(psa_abcdDSP(1:nbparamtotal),xApzmp1_anal,xBpzmp1_anal,tpassage,frequency);
        [ypzmp1 ytimezmp1prim]=zmp_generator_2(psa_abcdDSP(nbparamtotal+1:nbparamtotal+nbparamtotal),yApzmp1_anal,yBpzmp1_anal,tpassage,frequency);
    case {2,3}
        [xpzmp1 xtimezmp1prim]=zmp_generator_2(psa_abcdDSP(nbparamABCD+nbparamank+1:nbparamtotal),xApzmp1,xBpzmp1,tpassage,frequency);
        [ypzmp1 ytimezmp1prim]=zmp_generator_2(psa_abcdDSP(nbparamtotal+nbparamABCD+nbparamank+1:nbparamtotal+nbparamtotal),yApzmp1,yBpzmp1,tpassage,frequency);
end

switch(zmp_type)
    case {0,2,3,4,5,6,7,8}
        xpzmp2=xApzmp2*psa_abcdDSP(1:nbparamtotal)+xBpzmp2;
        ypzmp2=yApzmp2*psa_abcdDSP(nbparamtotal+1:nbparamtotal+nbparamtotal)+yBpzmp2;
    case 1
        [xpzmp2 xtimezmp2prim]=zmp_generator_2(psa_abcdDSP(1:nbparamtotal),xApzmp2_anal,xBpzmp2_anal,tpassage,frequency);
        [ypzmp2 ytimezmp2prim]=zmp_generator_2(psa_abcdDSP(nbparamtotal+1:nbparamtotal+nbparamtotal),yApzmp2_anal,yBpzmp2_anal,tpassage,frequency);
end


% xpzmp=trj(:,1);
% ypzmp=trj(:,2);

% %%%drawing xzmp(t)%%%
% figure(6)
% clf 
% axis auto
% title('x(t) of ZMP and COM (optimization on COM work)')
% xlabel('t(s)')
% ylabel('x(m)')
% hold on
% plot(xtimezmp,xpzmp)
% hold off
% %%%%%%%
% 
% %%%drawing yzmp(t)%%%
% figure(7)
% clf 
% axis auto
% title('y(t) of ZMP and COM (optimization on COM work)')
% xlabel('t[s]')
% ylabel('y[m]')
% hold on
% plot([0:length(ypzmp)-1]/200,ypzmp)
% hold off
% %%%%%%%

%%%drawing ZMP positions%%%
figure(8);
hold on;
plot(xpzmp,ypzmp,'-b','LineWidth',2);
% plot(xpzmp1,ypzmp1,'-*r')
% plot(xpzmp2,ypzmp2,'-*m')
plot(xpzmp1,ypzmp1,'*r')
plot(xpzmp2,ypzmp2,'*m')
legend('viapoints','ZMP')
hold off
%%%%%%%%%%%

%%%COM computation%%%
xpcom=xApcom*psa_abcd(1:end/2)+xBpcom;
ypcom=yApcom*psa_abcd(end/2+1:end)+yBpcom;
%%%%%%%%%%%

%%%drawing COM positions%%%
figure(8)
hold on
plot(xpcom,ypcom,'green','LineWidth',2);
% hleg = legend('ankle position','Via points','ZMP','COM','Location','NorthEast');
hleg = legend('ankle position','Via points','ZMP','ZMP1','ZMP2','COM','Location','EastOutside');
% Make the text of the legend italic and color it brown
set(hleg,'FontAngle','italic','TextColor',[.3,.2,.1])
hold off
%%%%%%%%%%%
% figure(6)
% hold on
% plot(time,xpcom,'green');
% hleg = legend('ZMP','COM','Location','Northwest');
% % Make the text of the legend italic and color it brown
% set(hleg,'FontAngle','italic','TextColor',[.3,.2,.1])
% hold off
% figure(7)
% hold on
% plot([0:length(ypcom)-1]/200,ypcom,'green');
% hleg = legend('ZMP','COM','Location','Northwest');
% % Make the text of the legend italic and color it brown
% set(hleg,'FontAngle','italic','TextColor',[.3,.2,.1])
% 
% plot([0:length(ypcom)-1]/200,pstep(1,2))
% plot([0:length(ypcom)-1]/200,pstep(2,2))
% hold off
% %%%%%%
end
%%%%%%

%%%drawing foot steps%%%


figure(8)
hold on;
XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle+toto,firstSS);
for i=1:length(pstep)
% for i=6:7
    plot(XY(i,1:5),XY(i,6:10),'-k','LineWidth',2)
end

XY=drawing_rectangle_rotate(pstep,psi,backtoankle-sole_margin,fronttoankle-sole_margin,exttoankle-sole_margin,inttoankle-sole_margin,firstSS);
for i=1:length(pstep)
% for i=6:7
    plot(XY(i,1:5),XY(i,6:10),':k','LineWidth',2)
end
hold off
%%%%%%%
figure(7)
clf 
axis auto
title('y(t) of ZMP and COM (optimization on COM work)')
xlabel('t[s]')
ylabel('y[m]')
hold on
plot([0:length(ypzmp)-1]/200,ypzmp)
toto=ypzmp.*any(dt_type_phase~=0,2);
plot([0:length(ypzmp)-1]/200,toto,'r')

plot([0:length(ypcom)-1]/200,ypcom,'green');
hleg = legend('ZMP','ZMP DSP','COM','Location','Northwest');
% Make the text of the legend italic and color it brown
set(hleg,'FontAngle','italic','TextColor',[.3,.2,.1])

plot([0:length(ypcom)-1]/200,pstep(1,2))
plot([0:length(ypcom)-1]/200,pstep(2,2))
hold off

figure(6)
clf 
axis auto
title('x(t) of ZMP and COM (optimization on COM work)')
xlabel('t[s]')
ylabel('x[m]')
hold on
plot([0:length(xpzmp)-1]/200,xpzmp)
toto=xpzmp.*any(dt_type_phase~=0,2);
plot([0:length(xpzmp)-1]/200,toto,'r')

plot([0:length(xpcom)-1]/200,xpcom,'green');
hleg = legend('ZMP','ZMP DSP','COM','Location','Northwest');
% Make the text of the legend italic and color it brown
set(hleg,'FontAngle','italic','TextColor',[.3,.2,.1])

% plot([0:length(xpcom)-1]/200,pstep(1,2))
% plot([0:length(xpcom)-1]/200,pstep(2,2))
hold off

open('AF_flexible_sole.m');
% open('AG_ankleDSP');
