function []=drawing(walking_param,display)
%draw ZMP, COM, ZMP1, ZMP2, foot step position in walking_param
%display :
%0 : don't display trajectories
%1 : display trajectories
load('trajectories.mat')

% xpzmp=[]; ypzmp=[]; xpcom=[]; ypcom=[];

switch(walking_param.zmp_type)
    case {0,4,5,6,7}
    case 1
walking_param.nbparamtotal=walking_param.nbparamABCD+walking_param.nbparamank;
    case {2,3}
walking_param.nbparamtotal=walking_param.nbparamABCD+walking_param.nbparamank+walking_param.nbparamBb;
    case {8}
        walking_param.nbparamABCD=(walking_param.nbstep*3+2+1)*3-6;
        walking_param.nbparamABCD=walking_param.nbparamABCD+4;
        walking_param.nbparamtotal=walking_param.nbparamABCD+walking_param.nbparamank+walking_param.nbparamBb;
end
%%
walking_param.psa_abcd=[walking_param.psa_abcdDSP(1:walking_param.nbparamABCD);walking_param.psa_abcdDSP(walking_param.nbparamtotal+1:walking_param.nbparamtotal+walking_param.nbparamABCD)];
%%
walking_param.pstep=[walking_param.psa_abcdDSP(walking_param.nbparamABCD+1:walking_param.nbparamABCD+walking_param.nbparamank) walking_param.psa_abcdDSP(walking_param.nbparamtotal+walking_param.nbparamABCD+1:walking_param.nbparamtotal+walking_param.nbparamABCD+walking_param.nbparamank)];
%%
walking_param.pstep=[walking_param.pankinit_firstinair;
    walking_param.pankinit_firstSS;
    walking_param.pstep;
    walking_param.pankfin_lastSS;
    walking_param.pankfin_lastinair];
%%
walking_param.pabcd=[walking_param.psa_abcd(1:3:walking_param.nbparamABCD-4) walking_param.psa_abcd(walking_param.nbparamABCD+1:3:walking_param.nbparamABCD+walking_param.nbparamABCD-4)];
%%%%%%%%%%
%%
if display
    %%%Clear figure(3) to draw y(x) coordinates%%%
    figure(8);
    clf;
    set(gca,'fontsize',14)
    axis equal;
    title('ZMP projected on the ground')
    xlabel('x(m)')
    ylabel('y(m)')
    hold on;
    plot(walking_param.pstep(:,1),walking_param.pstep(:,2),'+')
    plot(walking_param.pabcd(:,1),walking_param.pabcd(:,2),'o');
    hold off
    %%%%%%%%%%%
end
%% %%%using to draw zmp and com with new ZMP generator%%%

walking_param.xpzmp=xApzmp*walking_param.psa_abcd(1:length(walking_param.psa_abcd)/2)+xBpzmp;
walking_param.ypzmp=yApzmp*walking_param.psa_abcd(length(walking_param.psa_abcd)/2+1:end)+yBpzmp;

switch(walking_param.zmp_type)
    case {0,4,5,6,7,8}
        walking_param.xpzmp1=xApzmp1*walking_param.psa_abcdDSP(1:walking_param.nbparamtotal)+xBpzmp1;
        walking_param.ypzmp1=yApzmp1*walking_param.psa_abcdDSP(walking_param.nbparamtotal+1:walking_param.nbparamtotal+walking_param.nbparamtotal)+yBpzmp1;
    case 1
        [xpzmp1 xtimezmp1prim]=zmp_generator_2(psa_abcdDSP(1:nbparamtotal),xApzmp1_anal,xBpzmp1_anal,tpassage,frequency);
        [ypzmp1 ytimezmp1prim]=zmp_generator_2(psa_abcdDSP(nbparamtotal+1:nbparamtotal+nbparamtotal),yApzmp1_anal,yBpzmp1_anal,tpassage,frequency);
    case {2,3}
        [xpzmp1 xtimezmp1prim]=zmp_generator_2(psa_abcdDSP(nbparamABCD+nbparamank+1:nbparamtotal),xApzmp1,xBpzmp1,tpassage,frequency);
        [ypzmp1 ytimezmp1prim]=zmp_generator_2(psa_abcdDSP(nbparamtotal+nbparamABCD+nbparamank+1:nbparamtotal+nbparamtotal),yApzmp1,yBpzmp1,tpassage,frequency);
end

switch(walking_param.zmp_type)
    case {0,2,3,4,5,6,7,8}
        walking_param.xpzmp2=xApzmp2*walking_param.psa_abcdDSP(1:walking_param.nbparamtotal)+xBpzmp2;
        walking_param.ypzmp2=yApzmp2*walking_param.psa_abcdDSP(walking_param.nbparamtotal+1:walking_param.nbparamtotal+walking_param.nbparamtotal)+yBpzmp2;
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
% plot(ytimezmp,ypzmp)
% hold off
% %%%%%%%

if display
    %%%drawing ZMP positions%%%
    figure(8);
    hold on;
    plot(walking_param.xpzmp,walking_param.ypzmp,'-b','LineWidth',2);
    % plot(xpzmp1,ypzmp1,'-*r')
    % plot(xpzmp2,ypzmp2,'-*m')
    plot(walking_param.xpzmp1,walking_param.ypzmp1,'*r')
    plot(walking_param.xpzmp2,walking_param.ypzmp2,'*m')
    legend('viapoints','ZMP')
    hold off
    %%%%%%%%%%%
end

%%%COM computation%%%
walking_param.xpcom=xApcom*walking_param.psa_abcd(1:end/2)+xBpcom;
walking_param.ypcom=yApcom*walking_param.psa_abcd(end/2+1:end)+yBpcom;
%%%%%%%%%%%

if display
%%%drawing COM positions%%%
figure(8)
hold on
plot(walking_param.xpcom,walking_param.ypcom,'green','LineWidth',2);
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
% plot(time,ypcom,'green');
% hleg = legend('ZMP','COM','Location','Northwest');
% % Make the text of the legend italic and color it brown
% set(hleg,'FontAngle','italic','TextColor',[.3,.2,.1])
% hold off
% %%%%%%
%%%%%%
end

if display
%%%drawing foot steps%%%
figure(8)
hold on;
XY=drawing_rectangle_rotate(walking_param.pstep,walking_param.psi,walking_param.backtoankle,walking_param.fronttoankle,walking_param.exttoankle,walking_param.inttoankle,walking_param.firstSS);
for i=1:length(walking_param.pstep)
    plot(XY(i,1:5),XY(i,6:10),'-k','LineWidth',2)
end

XY=drawing_rectangle_rotate(walking_param.pstep,walking_param.psi,walking_param.backtoankle-walking_param.sole_margin,walking_param.fronttoankle-walking_param.sole_margin,walking_param.exttoankle-walking_param.sole_margin,walking_param.inttoankle-walking_param.sole_margin,walking_param.firstSS);
for i=1:length(walking_param.pstep)
    plot(XY(i,1:5),XY(i,6:10),':k','LineWidth',2)
end
hold off
%%%%%%%
end
end