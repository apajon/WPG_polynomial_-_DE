function []=drawing3D(walking_param,result_ankleDSP)
    load(result_ankleDSP);
    cutting=1/2;

    %% %%%Clear figure(9) to draw 3D%%%
figure(9);
clf;
view(3);
grid on;
% set(gca,'fontsize',14,'DataAspectRatioMode','auto','PlotBoxAspectRatio',[1 1 1])
set(gca,'fontsize',14,'DataAspectRatio',[1 1 1],'PlotBoxAspectRatio',[1 1 1])
% axis tight
% axis normal
% axis('DataAspectRatioMode','auto','PlotBoxAspectRatio',[1 1 1])
title('ZMP projected on the ground')
xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')
hold on;
% plot([walking_param.pstep(1,1);walking_param.pstep(:,1);walking_param.pstep(end,1)],[-walking_param.pstep(1,2);walking_param.pstep(:,2);walking_param.pstep(end,2)-(-1)^(nbstep)*0.095*2],'+')
plot3(walking_param.pstep(:,1),walking_param.pstep(:,2),ones(size(walking_param.pstep,1),1)*walking_param.ha,'+')
plot3(walking_param.pabcd(:,1),walking_param.pabcd(:,2),zeros(size(walking_param.pabcd,1),1),'o');
% legend('ankle positions','viapoints')
hold off
%%%%%%%%%%%
   
%% %%%using to draw zmp and com%%%
if 1
%% %%%drawing ZMP positions%%%
figure(9);
hold on;
plot3(walking_param.xpzmp,walking_param.ypzmp,zeros(size(walking_param.xpzmp,1),1),'-b','LineWidth',2);
% plot3(xpzmp1,ypzmp1,'-*r')
% plot3(xpzmp2,ypzmp2,'-*m')
plot3(walking_param.xpzmp1,walking_param.ypzmp1,zeros(size(walking_param.xpzmp1,1),1),'*c')
plot3(walking_param.xpzmp2,walking_param.ypzmp2,zeros(size(walking_param.xpzmp2,1),1),'*m')
% legend('ZMP','ZMP1','ZMP2')
hold off
%%%%%%%%%%%

%% %%%drawing COM positions%%%
figure(9)
hold on
plot3(walking_param.xpcom,walking_param.ypcom,zeros(size(walking_param.xpcom,1),1),'-k','LineWidth',2)
% plot3(xpcom,ypcom,ones(size(xpcom,1),1)*z,'-g','LineWidth',2)
% hleg = legend('ankle position','Via points','ZMP','COM','Location','NorthEast');
% hleg = legend('ankle position','Via points','ZMP','ZMP1','ZMP2','COM on the ground','COM','Location','EastOutside');
% hleg = legend('ankle position','Via points','ZMP','ZMP1','ZMP2','COM on the ground','Location','EastOutside');
% Make the text of the legend italic and color it brown
% set(hleg,'FontAngle','italic','TextColor',[.3,.2,.1])
hold off
%%%%%%%%%%%
end
%%%%%%

%% %%%Computation of in air foot angle%%%
dtangle=compute_coeff_dt_angle_matrix(walking_param.discretization,walking_param.frequency,walking_param.nbphases,walking_param.nbpointdiscret,walking_param.type_phase);
dtangle=dtangle(:,any(dtangle,1));

[Ac_modified]=compute_repartition_matrix_modified(walking_param.tpassage,walking_param.nbphases,cutting,walking_param.type_phase);
Ac_modified=Ac_modified(13:(walking_param.nbphases-sum(any(walking_param.type_phase==0,1)))*6+12);
[Afb_modified] = dtangle*Ac_modified;

rpsi_1_r_=rpsi_dt_r([1 round(walking_param.tpassage(2:end-1)/0.005)+2])';
rphi_1_r_=rphi_dt_r([1 round(walking_param.tpassage(2:end-1)/0.005)+2])';
rtheta_1_r_=rtheta_dt_r([1  round(walking_param.tpassage(2:end-1)/0.005)+2])';
rpsi_2_r_=rpsi_dt_r([ round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)])';
rphi_2_r_=rphi_dt_r([ round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)])';
rtheta_2_r_=rtheta_dt_r([ round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)])';

rpsi_1_r_=[0 rpsi_1_r_(3:end) 0];
rphi_1_r_=[0 rphi_1_r_(3:end) 0];
rtheta_1_r_=[0 rtheta_1_r_(3:end) 0];
rpsi_2_r_=[0 rpsi_2_r_(1:end-2) 0];
rphi_2_r_=[0 rphi_2_r_(1:end-2) 0];
rtheta_2_r_=[0 rtheta_2_r_(1:end-2) 0];

rpsi_1_l_=rpsi_dt_l([1  round(walking_param.tpassage(2:end-1)/0.005)+2])';
rphi_1_l_=rphi_dt_l([1  round(walking_param.tpassage(2:end-1)/0.005)+2])';
rtheta_1_l_=rtheta_dt_l([1 round(walking_param.tpassage(2:end-1)/0.005)+2])';
rpsi_2_l_=rpsi_dt_l([round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)])';
rphi_2_l_=rphi_dt_l([round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)])';
rtheta_2_l_=rtheta_dt_l([round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)])';

rpsi_1_l_=[0 rpsi_1_l_(3:end) 0];
rphi_1_l_=[0 rphi_1_l_(3:end) 0];
rtheta_1_l_=[0 rtheta_1_l_(3:end) 0];
rpsi_2_l_=[0 rpsi_2_l_(1:end-2) 0];
rphi_2_l_=[0 rphi_2_l_(1:end-2) 0];
rtheta_2_l_=[0 rtheta_2_l_(1:end-2) 0];


% rpsi_1=[ones(1,2)*walking_param.psi(1) ones(1,3)*walking_param.psi(3) walking_param.psi(4:end-4) walking_param.psi(end)];
% rpsi_2=[walking_param.psi(5:end) ones(1,3)*walking_param.psi(end)];

% rpsi_1(5:3:end-3)=(rpsi_1(5:3:end-3)+rpsi_2(5:3:end-3))/2;
% rpsi_2(4:3:end-4)=(rpsi_2(4:3:end-4)+rpsi_1(4:3:end-4))/2;

% rpsi_1_dt=compute_dt_type_phase(rpsi_1,discretization,nbphases,nbpointdiscret);
% rpsi_2_dt=compute_dt_type_phase(rpsi_2,discretization,nbphases,nbpointdiscret);

rpsi_1_dt_r=compute_dt_type_phase(rpsi_1_r_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
rpsi_2_dt_r=compute_dt_type_phase(rpsi_2_r_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
rphi_1_dt_r=compute_dt_type_phase(rphi_1_r_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
rphi_2_dt_r=compute_dt_type_phase(rphi_2_r_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
rtheta_1_dt_r=compute_dt_type_phase(rtheta_1_r_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
rtheta_2_dt_r=compute_dt_type_phase(rtheta_2_r_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);

rpsi_1_dt_l=compute_dt_type_phase(rpsi_1_l_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
rpsi_2_dt_l=compute_dt_type_phase(rpsi_2_l_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
rphi_1_dt_l=compute_dt_type_phase(rphi_1_l_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
rphi_2_dt_l=compute_dt_type_phase(rphi_2_l_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
rtheta_1_dt_l=compute_dt_type_phase(rtheta_1_l_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
rtheta_2_dt_l=compute_dt_type_phase(rtheta_2_l_,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);

% rpsi_dt=(rpsi_2_dt-rpsi_1_dt).*Afb_modified+rpsi_1_dt;

rpsi_dt_r_=(rpsi_1_dt_r-rpsi_2_dt_r).*(Afb_modified)+rpsi_2_dt_r;
rphi_dt_r_=(rphi_1_dt_r-rphi_2_dt_r).*(Afb_modified)+rphi_2_dt_r;
rtheta_dt_r_=(rtheta_1_dt_r-rtheta_2_dt_r).*(Afb_modified)+rtheta_2_dt_r;
rpsi_dt_r_(any(rpsi_dt_r~=0,2))=0;
rphi_dt_r_(any(rphi_dt_r~=0,2))=0;
rtheta_dt_r_(any(rtheta_dt_r~=0,2))=0;

rpsi_dt_l_=(rpsi_1_dt_l-rpsi_2_dt_l).*(Afb_modified)+rpsi_2_dt_l;
rphi_dt_l_=(rphi_1_dt_l-rphi_2_dt_l).*(Afb_modified)+rphi_2_dt_l;
rtheta_dt_l_=(rtheta_1_dt_l-rtheta_2_dt_l).*(Afb_modified)+rtheta_2_dt_l;
rpsi_dt_l_(any(rpsi_dt_l~=0,2))=0;
rphi_dt_l_(any(rphi_dt_l~=0,2))=0;
rtheta_dt_l_(any(rtheta_dt_l~=0,2))=0;


% rphi_1=zeros(1,size(rpsi_1,2));
% rphi_2=zeros(1,size(rpsi_1,2));
% rphi_1(5:3:end-3)=0.7854;
% rphi_2(4:3:end-4)=0.7854;
% rphi_1_dt=compute_dt_type_phase(rphi_1,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
% rphi_2_dt=compute_dt_type_phase(rphi_2,walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
% rphi_dt=(rphi_2_dt-rphi_1_dt).*Afb_modified+rphi_1_dt;

%% %%%computation of in-air foot trajectories
[xAinair_coeff_gradient]=compute_coeff_in_air(walking_param.tpassage,walking_param.nbphases,walking_param.type_phase);
yAinair_coeff_gradient=xAinair_coeff_gradient;
zAinair_coeff_gradient=xAinair_coeff_gradient;

dtpankle=compute_coeff_dt_pankle_matrix(walking_param.discretization,walking_param.frequency,walking_param.nbphases,walking_param.nbpointdiscret,walking_param.type_phase);
ddtpankle=compute_coeff_ddt_pankle_matrix(walking_param.discretization,walking_param.frequency,walking_param.nbphases,walking_param.nbpointdiscret,walking_param.type_phase);
dddtpankle=compute_coeff_dddt_pankle_matrix(walking_param.discretization,walking_param.frequency,walking_param.nbphases,walking_param.nbpointdiscret,walking_param.type_phase);
ddtpankle=ddtpankle(:,any(dtpankle,1));
dddtpankle=dddtpankle(:,any(dtpankle,1));
dtpankle=dtpankle(:,any(dtpankle,1));


xApinair=dtpankle*xAinair_coeff_gradient;
yApinair=dtpankle*yAinair_coeff_gradient;
zApinair=dtpankle*zAinair_coeff_gradient;

xAvinair=ddtpankle*xAinair_coeff_gradient;
yAvinair=ddtpankle*yAinair_coeff_gradient;
zAvinair=ddtpankle*zAinair_coeff_gradient;

xAainair=dddtpankle*xAinair_coeff_gradient;
yAainair=dddtpankle*yAinair_coeff_gradient;
zAainair=dddtpankle*zAinair_coeff_gradient;

pstep1_=[walking_param.pstep(1:end-2,:) ones(walking_param.nbpankle-2,1)*walking_param.ha];
pstep2_=[walking_param.pstep(3:end,:) ones(walking_param.nbpankle-2,1)*walking_param.ha];

%from 2 to 1
    pstep1_r=[xponfloor_r([1 round(walking_param.tpassage(2:end-1)/0.005)+2]) yponfloor_r([1 round(walking_param.tpassage(2:end-1)/0.005)+2]) zponfloor_r([1 round(walking_param.tpassage(2:end-1)/0.005)+2])];
    pstep1_r=pstep1_r(any(walking_param.type_phase'==0,2),:);
    pstep2_r=[xponfloor_r([round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)]) yponfloor_r([round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)]) zponfloor_r([round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)])];
    pstep2_r=pstep2_r(any(walking_param.type_phase'==0,2),:);

    pstep1_l=[xponfloor_l([1 round(walking_param.tpassage(2:end-1)/0.005)+2]) yponfloor_l([1 round(walking_param.tpassage(2:end-1)/0.005)+2]) zponfloor_l([1 round(walking_param.tpassage(2:end-1)/0.005)+2])];
    pstep1_l=pstep1_l(any(walking_param.type_phase'==0,2),:);
    pstep2_l=[xponfloor_l([round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)]) yponfloor_l([round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)]) zponfloor_l([round(walking_param.tpassage(2:end-1)/0.005)+1  round(walking_param.tpassage(end)/0.005)])];
    pstep2_l=pstep2_l(any(walking_param.type_phase'==0,2),:);

% if walking_param.firstSS==0
%      pstep1_r([1 end-1],:)=[];
%      pstep2_r([2 end],:)=[];
%      pstep1_l([1:2 end],:)=[];
%      pstep2_l([1 end-1:end],:)=[];
% elseif walking_param.firstSS==1
%      pstep1_l([1 end-1],:)=[];
%      pstep2_l([2 end],:)=[];
%      pstep1_r([1:2 end],:)=[];
%      pstep2_r([1 end-1:end],:)=[];
% else
%     'Choose a first SS foot'
% end
if walking_param.firstSS==0
     pstep1_r=pstep1_r(2:2:end,:);
     pstep2_r=pstep2_r(1:2:end,:);
     pstep1_l=pstep1_l(3:2:end-1,:);
     pstep2_l=pstep2_l(2:2:end-2,:);
elseif walking_param.firstSS==1
     pstep1_l= pstep1_l(2:2:end,:);
     pstep2_l=pstep2_l(1:2:end,:);
     pstep1_r=pstep1_r(3:2:end-1,:);
     pstep2_r=pstep2_r(2:2:end-2,:);
else
    'Choose a first SS foot'
end
%%
pstepm=zeros(walking_param.nbpankle-2,3);
for i=1:walking_param.nbpankle-2
    distank=norm((pstep1_(i,1:2)+pstep2_(i,1:2))/2-walking_param.pstep(i+1,1:2));
    threshold_ankle=walking_param.fronttoankle*1.3;
    if distank>=threshold_ankle
        pstepm(i,:)=[(pstep1_(i,1:2)+pstep2_(i,1:2))/2 walking_param.he];
    else
        pstepm(i,:)=[walking_param.pstep(i+1,1:2)+((pstep1_(i,1:2)+pstep2_(i,1:2))/2-walking_param.pstep(i+1,1:2))*threshold_ankle/distank walking_param.he];
    end

end

pstepm_r=zeros(size(pstep1_r,1),3);
pstepm_l=zeros(size(pstep1_l,1),3);
threshold_ankle=walking_param.fronttoankle*1.3;
r=0;l=0;
for i=1:walking_param.nbpankle-2
    rightorleft_inair=(-1)^(walking_param.firstSS+i+1);%+1 right, -1 left
    if rightorleft_inair==1
        r=r+1;
        distank=norm((pstep1_r(r,1:2)+pstep2_r(r,1:2))/2-walking_param.pstep(i+1,1:2));
        if distank>=threshold_ankle
            pstepm_r(r,:)=[(pstep1_r(r,1:2)+pstep2_r(r,1:2))/2 walking_param.he];
        else
            pstepm_r(r,:)=[walking_param.pstep(i+1,1:2)+((pstep1_r(r,1:2)+pstep2_r(r,1:2))/2-walking_param.pstep(i+1,1:2))*threshold_ankle/distank walking_param.he];
        end
    elseif rightorleft_inair==-1
        l=l+1;
        distank=norm((pstep1_l(l,1:2)+pstep2_l(l,1:2))/2-walking_param.pstep(i+1,1:2));
        if distank>=threshold_ankle
            pstepm_l(l,:)=[(pstep1_l(l,1:2)+pstep2_l(l,1:2))/2 walking_param.he];
        else
            pstepm_l(l,:)=[walking_param.pstep(i+1,1:2)+((pstep1_l(l,1:2)+pstep2_l(l,1:2))/2-walking_param.pstep(i+1,1:2))*threshold_ankle/distank walking_param.he];
        end
    end

end
%%
% param_in_air_r=zeros((walking_param.nbpankle-2)*6,3);
% param_in_air_l=zeros((walking_param.nbpankle-2)*6,3);
% if walking_param.firstSS==0
%     for i=1:2:walking_param.nbpankle-2
%         param_in_air_r(1+(i-1)*6:6+(i-1)*6,:)=[pstep1_(i,:)-pstep1_(i,:);
%             0 0 0;
%             pstepm(i,:)-pstep1_(i,:);
%             3/2*(pstep2_(i,1:2)-pstep1_(i,1:2))/(walking_param.tss) 0;
%             pstep2_(i,:)-pstep1_(i,:);
%             0 0 0];
%     end
%     for i=2:2:walking_param.nbpankle-2
%         param_in_air_l(1+(i-1)*6:6+(i-1)*6,:)=[pstep1_(i,:)-pstep1_(i,:);
%             0 0 0;
%             pstepm(i,:)-pstep1_(i,:);
%             3/2*(pstep2_(i,1:2)-pstep1_(i,1:2))/(walking_param.tss) 0;
%             pstep2_(i,:)-pstep1_(i,:);
%             0 0 0];
%     end
% elseif walking_param.firstSS==1
%     for i=1:2:walking_param.nbpankle-2
%         param_in_air_l(1+(i-1)*6:6+(i-1)*6,:)=[pstep1_(i,:)-pstep1_(i,:);
%             0 0 0;
%             pstepm(i,:)-pstep1_(i,:);
%             3/2*(pstep2_(i,1:2)-pstep1_(i,1:2))/(walking_param.tss) 0;
%             pstep2_(i,:)-pstep1_(i,:);
%             0 0 0];
%     end
%     for i=2:2:walking_param.nbpankle-2
%         param_in_air_r(1+(i-1)*6:6+(i-1)*6,:)=[pstep1_(i,:)-pstep1_(i,:);
%             0 0 0;
%             pstepm(i,:)-pstep1_(i,:);
%             3/2*(pstep2_(i,1:2)-pstep1_(i,1:2))/(walking_param.tss) 0;
%             pstep2_(i,:)-pstep1_(i,:);
%             0 0 0];
%     end
% else
%     'Choose a first SS foot'
% end

param_in_air_r=zeros((walking_param.nbpankle-2)*9,3);
param_in_air_l=zeros((walking_param.nbpankle-2)*9,3);
r=0;l=0;
if walking_param.firstSS==0
    for i=1:2:walking_param.nbpankle-2
        r=r+1;
        param_in_air_r(1+(i-1)*9:9+(i-1)*9,:)=[pstep2_r(r,:)-pstep2_r(r,:);
            0 0 0;
            0 0 0;
            pstepm_r(r,:)-pstep2_r(r,:);
            3/2*(pstep1_r(r,1:2)-pstep2_r(r,1:2))/(walking_param.tss) 0;
            0 0 0;
            pstep1_r(r,:)-pstep2_r(r,:);
            0 0 0;
            0 0 0];
    end
    for i=2:2:walking_param.nbpankle-2
        l=l+1;
        param_in_air_l(1+(i-1)*9:9+(i-1)*9,:)=[pstep2_l(l,:)-pstep2_l(l,:);
            0 0 0;
            0 0 0;
            pstepm_l(l,:)-pstep2_l(l,:);
            3/2*(pstep1_l(l,1:2)-pstep2_l(l,1:2))/(walking_param.tss) 0;
            0 0 0;
            pstep1_l(l,:)-pstep2_l(l,:);
            0 0 0;
            0 0 0];
    end
elseif walking_param.firstSS==1
    for i=1:2:walking_param.nbpankle-2
        l=l+1;
        param_in_air_l(1+(i-1)*9:9+(i-1)*9,:)=[pstep2_l(r,:)-pstep2_l(l,:);
            0 0 0;
            0 0 0;
            pstepm_l(l,:)-pstep2_l(l,:);
            3/2*(pstep1_l(l,1:2)-pstep2_l(l,1:2))/(walking_param.tss) 0;
            0 0 0;
            pstep1_l(l,:)-pstep2_l(l,:);
            0 0 0;
            0 0 0];
    end
    for i=2:2:walking_param.nbpankle-2
        r=r+1;
        param_in_air_r(1+(i-1)*9:9+(i-1)*9,:)=[pstep2_r(r,:)-pstep2_r(r,:);
            0 0 0;
            0 0 0;
            pstepm_r(r,:)-pstep2_r(r,:);
            3/2*(pstep1_r(r,1:2)-pstep2_r(r,1:2))/(walking_param.tss) 0;
            0 0 0;
            pstep1_r(r,:)-pstep2_r(r,:);
            0 0 0;
            0 0 0];
    end
else
    'Choose a first SS foot'
end

%%
% pstep1_r=zeros(size(walking_param.type_phase,2),3);
% pstep1_l=zeros(size(walking_param.type_phase,2),3);
% j=2;
% for i=3:6:size(walking_param.type_phase,2)
%     j=j+2;
%     pstep1_r(i:i+5,:)=[ones(6,1).*pstep1_(j-1,1) ones(6,1).*pstep1_(j-1,2) ones(6,1).*pstep1_(j-1,3)];
%     pstep1_l(i-2:i+5-2,:)=[ones(6,1).*pstep1_(j-2,1) ones(6,1).*pstep1_(j-2,2) ones(6,1).*pstep1_(j-2,3)];
% end
% pstep1_r(1:2,:)=[pstep1_(1,:);pstep1_(1,:)];
% pstep1_l(end-1:end,:)=[pstep1_(end,:);pstep1_(end,:)];
% pstep1_r_dt=compute_dt_walking_param.type_phase(pstep1_r',walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
% pstep1_l_dt=compute_dt_walking_param.type_phase(pstep1_l',walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);

pstep2_r_=zeros(size(walking_param.type_phase,2),3);
pstep2_l_=zeros(size(walking_param.type_phase,2),3);
r=0;l=0;
for i=1:size(walking_param.type_phase,2)
    if walking_param.type_phase(i)==1||i==2
        if walking_param.firstSS==0
            if r==l
                r=r+1;
                pstep2_r_(i,:)=pstep2_r(r,:);
            else
                l=l+1;
                pstep2_l_(i,:)=pstep2_l(l,:);
            end
        elseif walking_param.firstSS==1
            if r==l
                l=l+1;
                pstep2_l_(i,:)=pstep2_l(l,:);
            else
                r=r+1;
                pstep2_r_(i,:)=pstep2_r(r,:);
            end
        end
    elseif walking_param.type_phase(i)==2&&i~=2
        if walking_param.firstSS==0
            if r==l
                pstep2_l_(i,:)=pstep2_l(l,:);
            else
                pstep2_r_(i,:)=pstep2_r(r,:);
            end
        elseif walking_param.firstSS==1
            if r==l
                pstep2_r_(i,:)=pstep2_r(r,:);
            else
                pstep2_l_(i,:)=pstep2_l(l,:);
            end
        end
    end
end
    
pstep2_r_dt=compute_dt_type_phase(pstep2_r_',walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);
pstep2_l_dt=compute_dt_type_phase(pstep2_l_',walking_param.discretization,walking_param.nbphases,walking_param.nbpointdiscret);

%% %foot in air velocity
xvinair_r=xAvinair*param_in_air_r(:,1);
% yvinair_r=yAvinair*param_in_air_r(:,2);
% zvinair_r=zAvinair*param_in_air_r(:,3);
% xvinair_r=xvinair_r.*any(walking_param.dt_type_phase~=0,2);
% yvinair_r=yvinair_r.*any(walking_param.dt_type_phase~=0,2);
% zvinair_r=zvinair_r.*any(walking_param.dt_type_phase~=0,2);
% 
xvinair_l=xAvinair*param_in_air_l(:,1);
% yvinair_l=yAvinair*param_in_air_l(:,2);
% zvinair_l=zAvinair*param_in_air_l(:,3);
% xvinair_l=xvinair_l.*any(walking_param.dt_type_phase~=0,2);
% yvinair_l=yvinair_l.*any(walking_param.dt_type_phase~=0,2);
% zvinair_l=zvinair_l.*any(walking_param.dt_type_phase~=0,2);

% %% %foot in air acceleration
% xainair_r=xAainair*param_in_air_r(:,1);
% yainair_r=yAainair*param_in_air_r(:,2);
% zainair_r=zAainair*param_in_air_r(:,3);
% xainair_r=xainair_r.*any(walking_param.dt_type_phase~=0,2);
% yainair_r=yainair_r.*any(walking_param.dt_type_phase~=0,2);
% zainair_r=zainair_r.*any(walking_param.dt_type_phase~=0,2);
% 
% xainair_l=xAainair*param_in_air_l(:,1);
% yainair_l=yAainair*param_in_air_l(:,2);
% zainair_l=zAainair*param_in_air_l(:,3);
% xainair_l=xainair_l.*any(walking_param.dt_type_phase~=0,2);
% yainair_l=yainair_l.*any(walking_param.dt_type_phase~=0,2);
% zainair_l=zainair_l.*any(walking_param.dt_type_phase~=0,2);

%% %foot in air position
% xpinair_r=xApinair*param_in_air_r(:,1)+pstep1_r_dt(:,1);
% ypinair_r=yApinair*param_in_air_r(:,2)+pstep1_r_dt(:,2);
% zpinair_r=zApinair*param_in_air_r(:,3)+pstep1_r_dt(:,3);
% xpinair_r=xpinair_r.*any(xvinair_r,2);
% ypinair_r=ypinair_r.*any(yvinair_r,2);
% zpinair_r=zpinair_r.*any(yvinair_r,2);
% xpinair_r_=xpinair_r(any(walking_param.dt_type_phase~=0,2),:);
% ypinair_r_=ypinair_r(any(walking_param.dt_type_phase~=0,2),:);
% zpinair_r_=zpinair_r(any(walking_param.dt_type_phase~=0,2),:);
% 
% xpinair_l=xApinair*param_in_air_l(:,1)+pstep1_l_dt(:,1);
% ypinair_l=yApinair*param_in_air_l(:,2)+pstep1_l_dt(:,2);
% zpinair_l=zApinair*param_in_air_l(:,3)+pstep1_l_dt(:,3);
% xpinair_l=xpinair_l.*any(xvinair_l,2);
% ypinair_l=ypinair_l.*any(yvinair_l,2);
% zpinair_l=zpinair_l.*any(zvinair_l,2);
% xpinair_l_=xpinair_l(any(walking_param.dt_type_phase~=0,2),:);
% ypinair_l_=ypinair_l(any(walking_param.dt_type_phase~=0,2),:);
% zpinair_l_=zpinair_l(any(walking_param.dt_type_phase~=0,2),:);

xpinair_r=xApinair*param_in_air_r(:,1)+pstep2_r_dt(:,1);
ypinair_r=yApinair*param_in_air_r(:,2)+pstep2_r_dt(:,2);
zpinair_r=zApinair*param_in_air_r(:,3)+pstep2_r_dt(:,3);
% xpinair_r=xpinair_r.*any(xvinair_r,2);
% ypinair_r=ypinair_r.*any(yvinair_r,2);
% zpinair_r=zpinair_r.*any(yvinair_r,2);
% xpinair_r_=xpinair_r(any(walking_param.dt_type_phase~=0,2),:);
% ypinair_r_=ypinair_r(any(walking_param.dt_type_phase~=0,2),:);
% zpinair_r_=zpinair_r(any(walking_param.dt_type_phase~=0,2),:);

xpinair_l=xApinair*param_in_air_l(:,1)+pstep2_l_dt(:,1);
ypinair_l=yApinair*param_in_air_l(:,2)+pstep2_l_dt(:,2);
zpinair_l=zApinair*param_in_air_l(:,3)+pstep2_l_dt(:,3);
% xpinair_l=xApinair*param_in_air_l(:,1);
% ypinair_l=yApinair*param_in_air_l(:,2);
% zpinair_l=zApinair*param_in_air_l(:,3);

% xpinair_l=xpinair_l.*any(xvinair_l,2);
% ypinair_l=ypinair_l.*any(yvinair_l,2);
% zpinair_l=zpinair_l.*any(zvinair_l,2);
% xpinair_l_=xpinair_l(any(walking_param.dt_type_phase~=0,2),:);
% ypinair_l_=ypinair_l(any(walking_param.dt_type_phase~=0,2),:);
% zpinair_l_=zpinair_l(any(walking_param.dt_type_phase~=0,2),:);


%% %repartition of in air angle

% rpsi_dt_l=rpsi_dt.*any(xvinair_l,2);
% rpsi_dt_r=rpsi_dt.*any(xvinair_r,2);
%%%%%%%

%%
figure(9)
hold on
% plot3(xpinair_,ypinair_,zpinair_,'+')
plot3(xpinair_r,ypinair_r,zpinair_r,'+g','LineWidth',4)
plot3(xpinair_l,ypinair_l,zpinair_l,'+r','LineWidth',4)
hleg = legend('ankle position','Via points','ZMP','ZMP1','ZMP2','COM on the ground','right ankle','left ankle','Location','EastOutside');
hold off

%% %%%drawing foot steps%%%
figure(9)
hold on;
% for i=1:size(walking_param.pstep,1)
% %     rectangle('Position',[walking_param.pstep(i,1)-walking_param.backtoankle,walking_param.pstep(i,2)-walking_param.inttoankle*mod(i,2)-walking_param.exttoankle*mod(i+1,2),b,a],'EdgeColor','black','LineStyle',':','LineWidth',2)
%     rectangle('Position',[walking_param.pstep(i,1)-walking_param.backtoankle,walking_param.pstep(i,2)-walking_param.inttoankle*mod(i+1,2)-walking_param.exttoankle*mod(i,2),b,a],'EdgeColor','black','LineStyle',':','LineWidth',2)
% 
% end
% rectangle('Position',[walking_param.pstep(1,1)-walking_param.backtoankle,-walking_param.pstep(1,2)-walking_param.inttoankle*mod(2,2)-walking_param.exttoankle*mod(2+1,2),b,a],'EdgeColor','black','LineStyle',':','LineWidth',2)
% rectangle('Position',[walking_param.pstep(size(walking_param.pstep,1),1)-walking_param.backtoankle,walking_param.pstep(size(walking_param.pstep,1),2)-(-1)^(nbstep)*0.095*2-walking_param.inttoankle*mod(size(walking_param.pstep,1)+1,2)-walking_param.exttoankle*mod(size(walking_param.pstep,1)+1+1,2),b,a],'EdgeColor','black','LineStyle',':','LineWidth',2)
XY=drawing_rectangle_rotate(walking_param.pstep,walking_param.psi,walking_param.backtoankle,walking_param.fronttoankle,walking_param.exttoankle,walking_param.inttoankle,walking_param.firstSS);
for i=1:length(walking_param.pstep)
    plot3(XY(i,1:5),XY(i,6:10),zeros(1,5),'-k','LineWidth',2)
end

XY=drawing_rectangle_rotate(walking_param.pstep,walking_param.psi,walking_param.backtoankle-walking_param.sole_margin,walking_param.fronttoankle-walking_param.sole_margin,walking_param.exttoankle-walking_param.sole_margin,walking_param.inttoankle-walking_param.sole_margin,walking_param.firstSS);
for i=1:length(walking_param.pstep)
    plot3(XY(i,1:5),XY(i,6:10),zeros(1,5),':k','LineWidth',2)
end
hold off
%%%%%%%
    
%%
walking_param.xpankle_r=walking_param.xpankle_r+xpinair_r;
walking_param.ypankle_r=walking_param.ypankle_r+ypinair_r;
walking_param.zpankle_r=walking_param.zpankle_r+zpinair_r;
walking_param.xpankle_l=walking_param.xpankle_l+xpinair_l;
walking_param.ypankle_l=walking_param.ypankle_l+ypinair_l;
walking_param.zpankle_l=walking_param.zpankle_l+zpinair_l;
walking_param.rpsi_r=walking_param.rpsi_r+rpsi_dt_r_;
walking_param.rphi_r=walking_param.rphi_r+rphi_dt_r_;
walking_param.rtheta_r=walking_param.rtheta_l+rtheta_dt_r_;
walking_param.rphi_l=walking_param.rphi_l+rpsi_dt_l_;
walking_param.rpsi_l=walking_param.rpsi_l+rphi_dt_l_;
walking_param.rtheta_l=walking_param.rtheta_l+rtheta_dt_l_;

%% %recompute dt_type_phase
dt_type_phase_=any(xvinair_l,2)*2+any(xvinair_r,2)*1;
save('generalized_functions/support_foot.mat','dt_type_phase_');
end