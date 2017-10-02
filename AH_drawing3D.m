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

%% %%%Clear figure(9) to draw 3D%%%
figure(9);
clf;
view(3);
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
% plot([pstep(1,1);pstep(:,1);pstep(end,1)],[-pstep(1,2);pstep(:,2);pstep(end,2)-(-1)^(nbstep)*0.095*2],'+')
plot3(pstep(:,1),pstep(:,2),ones(size(pstep,1),1)*(ha+(e-e_)),'+')
plot3(pabcd(:,1),pabcd(:,2),zeros(size(pabcd,1),1),'o');
% legend('ankle positions','viapoints')
hold off
%%%%%%%%%%%
   
%% %%%using to draw zmp and com%%%
if 1
%% %%%drawing ZMP positions%%%
figure(9);
hold on;
plot3(xpzmp,ypzmp,zeros(size(xpzmp,1),1),'-b','LineWidth',2);
% plot3(xpzmp1,ypzmp1,'-*r')
% plot3(xpzmp2,ypzmp2,'-*m')
plot3(xpzmp1,ypzmp1,zeros(size(xpzmp1,1),1),'*c')
plot3(xpzmp2,ypzmp2,zeros(size(xpzmp2,1),1),'*m')
% legend('ZMP','ZMP1','ZMP2')
hold off
%%%%%%%%%%%

%% %%%drawing COM positions%%%
figure(9)
hold on
plot3(xpcom,ypcom,zeros(size(xpcom,1),1),'-k','LineWidth',2)
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
dtangle=compute_coeff_dt_angle_matrix(discretization,frequency,nbphases,nbpointdiscret,type_phase);
dtangle=dtangle(:,any(dtangle,1));

[Ac_modified]=compute_repartition_matrix_modified(tpassage,nbphases,cutting,type_phase);
Ac_modified=Ac_modified(13:(nbphases-sum(any(type_phase==0,1)))*6+12);
[Afb_modified] = dtangle*Ac_modified;

rpsi_1_r_=rpsi_dt_r([1 round(tpassage(2:end-1)/0.005)+2])';
rphi_1_r_=rphi_dt_r([1 round(tpassage(2:end-1)/0.005)+2])';
rtheta_1_r_=rtheta_dt_r([1  round(tpassage(2:end-1)/0.005)+2])';
rpsi_2_r_=rpsi_dt_r([ round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)])';
rphi_2_r_=rphi_dt_r([ round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)])';
rtheta_2_r_=rtheta_dt_r([ round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)])';

% rpsi_1_r_([7 8 13 14 19 20 25 26])=[rpsi_2_r_(6) (rpsi_2_r_(6)+rpsi_1_r_(9))/2 rpsi_2_r_(12) (rpsi_2_r_(12)+rpsi_1_r_(15))/2 rpsi_2_r_(18) (rpsi_2_r_(18)+rpsi_1_r_(21))/2 rpsi_2_r_(24) (rpsi_2_r_(24)+rpsi_1_r_(27))/2];
% rphi_1_r_([7 8 13 14 19 20 25 26])=[rphi_2_r_(6) (rphi_2_r_(6)+rphi_1_r_(9))/2 rphi_2_r_(12) (rphi_2_r_(12)+rphi_1_r_(15))/2 rphi_2_r_(18) (rphi_2_r_(18)+rphi_1_r_(21))/2 rphi_2_r_(24) (rphi_2_r_(24)+rphi_1_r_(27))/2];
% rtheta_1_r_([7 8 13 14 19 20 25 26])=[rtheta_2_r_(6) (rtheta_2_r_(6)+rtheta_1_r_(9))/2 rtheta_2_r_(12) (rtheta_2_r_(12)+rtheta_1_r_(15))/2 rtheta_2_r_(18) (rtheta_2_r_(18)+rtheta_1_r_(21))/2 rtheta_2_r_(24) (rtheta_2_r_(24)+rtheta_1_r_(27))/2];
% rpsi_2_r_([7 8 13 14 19 20 25 26])=[(rpsi_2_r_(6)+rpsi_1_r_(9))/2 rpsi_1_r_(9) (rpsi_2_r_(12)+rpsi_1_r_(15))/2 rpsi_1_r_(15) (rpsi_2_r_(18)+rpsi_1_r_(21))/2 rpsi_1_r_(21) (rpsi_2_r_(24)+rpsi_1_r_(27))/2 rpsi_1_r_(27)];
% rphi_2_r_([7 8 13 14 19 20 25 26])=[(rphi_2_r_(6)+rphi_1_r_(9))/2 rphi_1_r_(9) (rphi_2_r_(12)+rphi_1_r_(15))/2 rphi_1_r_(15) (rphi_2_r_(18)+rphi_1_r_(21))/2 rphi_1_r_(21) (rphi_2_r_(24)+rphi_1_r_(27))/2 rphi_1_r_(27)];
% rtheta_2_r_([7 8 13 14 19 20 25 26])=[(rtheta_2_r_(6)+rtheta_1_r_(9))/2 rtheta_1_r_(9) (rtheta_2_r_(12)+rtheta_1_r_(15))/2 rtheta_1_r_(15) (rtheta_2_r_(18)+rtheta_1_r_(21))/2 rtheta_1_r_(21) (rtheta_2_r_(24)+rtheta_1_r_(27))/2 rtheta_1_r_(27)];

rpsi_1_r_=[0 rpsi_1_r_(3:end) 0];
rphi_1_r_=[0 rphi_1_r_(3:end) 0];
rtheta_1_r_=[0 rtheta_1_r_(3:end) 0];
rpsi_2_r_=[0 rpsi_2_r_(1:end-2) 0];
rphi_2_r_=[0 rphi_2_r_(1:end-2) 0];
rtheta_2_r_=[0 rtheta_2_r_(1:end-2) 0];

rpsi_1_l_=rpsi_dt_l([1  round(tpassage(2:end-1)/0.005)+2])';
rphi_1_l_=rphi_dt_l([1  round(tpassage(2:end-1)/0.005)+2])';
rtheta_1_l_=rtheta_dt_l([1 round(tpassage(2:end-1)/0.005)+2])';
rpsi_2_l_=rpsi_dt_l([round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)])';
rphi_2_l_=rphi_dt_l([round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)])';
rtheta_2_l_=rtheta_dt_l([round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)])';

% rpsi_1_l_([4 5 10 11 16 17 22 23 28 29])=[rpsi_2_l_(3) (rpsi_2_l_(3)+rpsi_1_l_(6))/2 rpsi_2_l_(9) (rpsi_2_l_(9)+rpsi_1_l_(12))/2 rpsi_2_l_(15) (rpsi_2_l_(15)+rpsi_1_l_(18))/2 rpsi_2_l_(21) (rpsi_2_l_(21)+rpsi_1_l_(24))/2 rpsi_2_l_(26) (rpsi_2_l_(26)+rpsi_1_l_(30))/2];
% rphi_1_l_([4 5 10 11 16 17 22 23 28 29])=[rphi_2_l_(3) (rphi_2_l_(3)+rphi_1_l_(6))/2 rphi_2_l_(9) (rphi_2_l_(9)+rphi_1_l_(12))/2 rphi_2_l_(15) (rphi_2_l_(15)+rphi_1_l_(18))/2 rphi_2_l_(21) (rphi_2_l_(21)+rphi_1_l_(24))/2 rphi_2_l_(26) (rphi_2_l_(26)+rphi_1_l_(30))/2];
% rtheta_1_l_([4 5 10 11 16 17 22 23 28 29])=[rtheta_2_l_(3) (rtheta_2_l_(3)+rtheta_1_l_(6))/2 rtheta_2_l_(9) (rtheta_2_l_(9)+rtheta_1_l_(12))/2 rtheta_2_l_(15) (rtheta_2_l_(15)+rtheta_1_l_(18))/2 rtheta_2_l_(21) (rtheta_2_l_(21)+rtheta_1_l_(24))/2 rtheta_2_l_(26) (rtheta_2_l_(26)+rtheta_1_l_(30))/2];
% rpsi_2_l_([4 5 10 11 16 17 22 23 28 29])=[(rpsi_2_l_(3)+rpsi_1_l_(6))/2 rpsi_1_l_(6) (rpsi_2_l_(9)+rpsi_1_l_(12))/2 rpsi_1_l_(12) (rpsi_2_l_(15)+rpsi_1_l_(18))/2 rpsi_1_l_(18) (rpsi_2_l_(21)+rpsi_1_l_(24))/2 rpsi_1_l_(24) (rpsi_2_l_(26)+rpsi_1_l_(30))/2 rpsi_1_l_(30)];
% rphi_2_l_([4 5 10 11 16 17 22 23 28 29])=[(rphi_2_l_(3)+rphi_1_l_(6))/2 rphi_1_l_(6) (rphi_2_l_(9)+rphi_1_l_(12))/2 rphi_1_l_(12) (rphi_2_l_(15)+rphi_1_l_(18))/2 rphi_1_l_(18) (rphi_2_l_(21)+rphi_1_l_(24))/2 rphi_1_l_(24) (rphi_2_l_(26)+rphi_1_l_(30))/2 rphi_1_l_(30)];
% rtheta_2_l_([4 5 10 11 16 17 22 23 28 29])=[(rtheta_2_l_(3)+rtheta_1_l_(6))/2 rtheta_1_l_(6) (rtheta_2_l_(9)+rtheta_1_l_(12))/2 rtheta_1_l_(12) (rtheta_2_l_(15)+rtheta_1_l_(18))/2 rtheta_1_l_(18) (rtheta_2_l_(21)+rtheta_1_l_(24))/2 rtheta_1_l_(24) (rtheta_2_l_(26)+rtheta_1_l_(30))/2 rtheta_1_l_(30)];

rpsi_1_l_=[0 rpsi_1_l_(3:end) 0];
rphi_1_l_=[0 rphi_1_l_(3:end) 0];
rtheta_1_l_=[0 rtheta_1_l_(3:end) 0];
rpsi_2_l_=[0 rpsi_2_l_(1:end-2) 0];
rphi_2_l_=[0 rphi_2_l_(1:end-2) 0];
rtheta_2_l_=[0 rtheta_2_l_(1:end-2) 0];


% rpsi_1=[ones(1,2)*psi(1) ones(1,3)*psi(3) psi(4:end-4) psi(end)];
% rpsi_2=[psi(5:end) ones(1,3)*psi(end)];

% rpsi_1(5:3:end-3)=(rpsi_1(5:3:end-3)+rpsi_2(5:3:end-3))/2;
% rpsi_2(4:3:end-4)=(rpsi_2(4:3:end-4)+rpsi_1(4:3:end-4))/2;

% rpsi_1_dt=compute_dt_type_phase(rpsi_1,discretization,nbphases,nbpointdiscret);
% rpsi_2_dt=compute_dt_type_phase(rpsi_2,discretization,nbphases,nbpointdiscret);

rpsi_1_dt_r=compute_dt_type_phase(rpsi_1_r_,discretization,nbphases,nbpointdiscret);
rpsi_2_dt_r=compute_dt_type_phase(rpsi_2_r_,discretization,nbphases,nbpointdiscret);
rphi_1_dt_r=compute_dt_type_phase(rphi_1_r_,discretization,nbphases,nbpointdiscret);
rphi_2_dt_r=compute_dt_type_phase(rphi_2_r_,discretization,nbphases,nbpointdiscret);
rtheta_1_dt_r=compute_dt_type_phase(rtheta_1_r_,discretization,nbphases,nbpointdiscret);
rtheta_2_dt_r=compute_dt_type_phase(rtheta_2_r_,discretization,nbphases,nbpointdiscret);

rpsi_1_dt_l=compute_dt_type_phase(rpsi_1_l_,discretization,nbphases,nbpointdiscret);
rpsi_2_dt_l=compute_dt_type_phase(rpsi_2_l_,discretization,nbphases,nbpointdiscret);
rphi_1_dt_l=compute_dt_type_phase(rphi_1_l_,discretization,nbphases,nbpointdiscret);
rphi_2_dt_l=compute_dt_type_phase(rphi_2_l_,discretization,nbphases,nbpointdiscret);
rtheta_1_dt_l=compute_dt_type_phase(rtheta_1_l_,discretization,nbphases,nbpointdiscret);
rtheta_2_dt_l=compute_dt_type_phase(rtheta_2_l_,discretization,nbphases,nbpointdiscret);

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
% rphi_1_dt=compute_dt_type_phase(rphi_1,discretization,nbphases,nbpointdiscret);
% rphi_2_dt=compute_dt_type_phase(rphi_2,discretization,nbphases,nbpointdiscret);
% rphi_dt=(rphi_2_dt-rphi_1_dt).*Afb_modified+rphi_1_dt;

%% %%%computation of in-air foot trajectories
[xAinair_coeff_gradient]=compute_coeff_in_air(tpassage,nbphases,type_phase);
yAinair_coeff_gradient=xAinair_coeff_gradient;
zAinair_coeff_gradient=xAinair_coeff_gradient;

dtpankle=compute_coeff_dt_pankle_matrix(discretization,frequency,nbphases,nbpointdiscret,type_phase);
ddtpankle=compute_coeff_ddt_pankle_matrix(discretization,frequency,nbphases,nbpointdiscret,type_phase);
dddtpankle=compute_coeff_dddt_pankle_matrix(discretization,frequency,nbphases,nbpointdiscret,type_phase);
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

pstep1_=[pstep(1:end-2,:) ones(nbpankle-2,1)*(ha+(e-e_))];
pstep2_=[pstep(3:end,:) ones(nbpankle-2,1)*(ha+(e-e_))];

%from 2 to 1
    pstep1_r=[xponfloor_r([1 round(tpassage(2:end-1)/0.005)+2]) yponfloor_r([1 round(tpassage(2:end-1)/0.005)+2]) zponfloor_r([1 round(tpassage(2:end-1)/0.005)+2])];
    pstep1_r=pstep1_r(any(type_phase'==0,2),:);
    pstep2_r=[xponfloor_r([round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)]) yponfloor_r([round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)]) zponfloor_r([round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)])];
    pstep2_r=pstep2_r(any(type_phase'==0,2),:);

    pstep1_l=[xponfloor_l([1 round(tpassage(2:end-1)/0.005)+2]) yponfloor_l([1 round(tpassage(2:end-1)/0.005)+2]) zponfloor_l([1 round(tpassage(2:end-1)/0.005)+2])];
    pstep1_l=pstep1_l(any(type_phase'==0,2),:);
    pstep2_l=[xponfloor_l([round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)]) yponfloor_l([round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)]) zponfloor_l([round(tpassage(2:end-1)/0.005)+1  round(tpassage(end)/0.005)])];
    pstep2_l=pstep2_l(any(type_phase'==0,2),:);

% if firstSS==0
%      pstep1_r([1 end-1],:)=[];
%      pstep2_r([2 end],:)=[];
%      pstep1_l([1:2 end],:)=[];
%      pstep2_l([1 end-1:end],:)=[];
% elseif firstSS==1
%      pstep1_l([1 end-1],:)=[];
%      pstep2_l([2 end],:)=[];
%      pstep1_r([1:2 end],:)=[];
%      pstep2_r([1 end-1:end],:)=[];
% else
%     'Choose a first SS foot'
% end
if firstSS==0
     pstep1_r=pstep1_r(2:2:end,:);
     pstep2_r=pstep2_r(1:2:end,:);
     pstep1_l=pstep1_l(3:2:end-1,:);
     pstep2_l=pstep2_l(2:2:end-2,:);
elseif firstSS==1
     pstep1_l= pstep1_l(2:2:end,:);
     pstep2_l=pstep2_l(1:2:end,:);
     pstep1_r=pstep1_r(3:2:end-1,:);
     pstep2_r=pstep2_r(2:2:end-2,:);
else
    'Choose a first SS foot'
end
%%
pstepm=zeros(nbpankle-2,3);
for i=1:nbpankle-2
    distank=norm((pstep1_(i,1:2)+pstep2_(i,1:2))/2-pstep(i+1,1:2));
    threshold_ankle=fronttoankle*1.3;
    if distank>=threshold_ankle
        pstepm(i,:)=[(pstep1_(i,1:2)+pstep2_(i,1:2))/2 he];
    else
        pstepm(i,:)=[pstep(i+1,1:2)+((pstep1_(i,1:2)+pstep2_(i,1:2))/2-pstep(i+1,1:2))*threshold_ankle/distank he];
    end

end

pstepm_r=zeros(size(pstep1_r,1),3);
pstepm_l=zeros(size(pstep1_l,1),3);
threshold_ankle=fronttoankle*1.3;
r=0;l=0;
for i=1:nbpankle-2
    rightorleft_inair=(-1)^(firstSS+i+1);%+1 right, -1 left
    if rightorleft_inair==1
        r=r+1;
        distank=norm((pstep1_r(r,1:2)+pstep2_r(r,1:2))/2-pstep(i+1,1:2));
        if distank>=threshold_ankle
            pstepm_r(r,:)=[(pstep1_r(r,1:2)+pstep2_r(r,1:2))/2 he];
        else
            pstepm_r(r,:)=[pstep(i+1,1:2)+((pstep1_r(r,1:2)+pstep2_r(r,1:2))/2-pstep(i+1,1:2))*threshold_ankle/distank he];
        end
    elseif rightorleft_inair==-1
        l=l+1;
        distank=norm((pstep1_l(l,1:2)+pstep2_l(l,1:2))/2-pstep(i+1,1:2));
        if distank>=threshold_ankle
            pstepm_l(l,:)=[(pstep1_l(l,1:2)+pstep2_l(l,1:2))/2 he];
        else
            pstepm_l(l,:)=[pstep(i+1,1:2)+((pstep1_l(l,1:2)+pstep2_l(l,1:2))/2-pstep(i+1,1:2))*threshold_ankle/distank he];
        end
    end

end
%%
% param_in_air_r=zeros((nbpankle-2)*6,3);
% param_in_air_l=zeros((nbpankle-2)*6,3);
% if firstSS==0
%     for i=1:2:nbpankle-2
%         param_in_air_r(1+(i-1)*6:6+(i-1)*6,:)=[pstep1_(i,:)-pstep1_(i,:);
%             0 0 0;
%             pstepm(i,:)-pstep1_(i,:);
%             3/2*(pstep2_(i,1:2)-pstep1_(i,1:2))/(tss) 0;
%             pstep2_(i,:)-pstep1_(i,:);
%             0 0 0];
%     end
%     for i=2:2:nbpankle-2
%         param_in_air_l(1+(i-1)*6:6+(i-1)*6,:)=[pstep1_(i,:)-pstep1_(i,:);
%             0 0 0;
%             pstepm(i,:)-pstep1_(i,:);
%             3/2*(pstep2_(i,1:2)-pstep1_(i,1:2))/(tss) 0;
%             pstep2_(i,:)-pstep1_(i,:);
%             0 0 0];
%     end
% elseif firstSS==1
%     for i=1:2:nbpankle-2
%         param_in_air_l(1+(i-1)*6:6+(i-1)*6,:)=[pstep1_(i,:)-pstep1_(i,:);
%             0 0 0;
%             pstepm(i,:)-pstep1_(i,:);
%             3/2*(pstep2_(i,1:2)-pstep1_(i,1:2))/(tss) 0;
%             pstep2_(i,:)-pstep1_(i,:);
%             0 0 0];
%     end
%     for i=2:2:nbpankle-2
%         param_in_air_r(1+(i-1)*6:6+(i-1)*6,:)=[pstep1_(i,:)-pstep1_(i,:);
%             0 0 0;
%             pstepm(i,:)-pstep1_(i,:);
%             3/2*(pstep2_(i,1:2)-pstep1_(i,1:2))/(tss) 0;
%             pstep2_(i,:)-pstep1_(i,:);
%             0 0 0];
%     end
% else
%     'Choose a first SS foot'
% end

param_in_air_r=zeros((nbpankle-2)*9,3);
param_in_air_l=zeros((nbpankle-2)*9,3);
r=0;l=0;
if firstSS==0
    for i=1:2:nbpankle-2
        r=r+1;
        param_in_air_r(1+(i-1)*9:9+(i-1)*9,:)=[pstep2_r(r,:)-pstep2_r(r,:);
            0 0 0;
            0 0 0;
            pstepm_r(r,:)-pstep2_r(r,:);
            3/2*(pstep1_r(r,1:2)-pstep2_r(r,1:2))/(tss) 0;
            0 0 0;
            pstep1_r(r,:)-pstep2_r(r,:);
            0 0 0;
            0 0 0];
    end
    for i=2:2:nbpankle-2
        l=l+1;
        param_in_air_l(1+(i-1)*9:9+(i-1)*9,:)=[pstep2_l(l,:)-pstep2_l(l,:);
            0 0 0;
            0 0 0;
            pstepm_l(l,:)-pstep2_l(l,:);
            3/2*(pstep1_l(l,1:2)-pstep2_l(l,1:2))/(tss) 0;
            0 0 0;
            pstep1_l(l,:)-pstep2_l(l,:);
            0 0 0;
            0 0 0];
    end
elseif firstSS==1
    for i=1:2:nbpankle-2
        l=l+1;
        param_in_air_l(1+(i-1)*9:9+(i-1)*9,:)=[pstep2_l(r,:)-pstep2_l(l,:);
            0 0 0;
            0 0 0;
            pstepm_l(l,:)-pstep2_l(l,:);
            3/2*(pstep1_l(l,1:2)-pstep2_l(l,1:2))/(tss) 0;
            0 0 0;
            pstep1_l(l,:)-pstep2_l(l,:);
            0 0 0;
            0 0 0];
    end
    for i=2:2:nbpankle-2
        r=r+1;
        param_in_air_r(1+(i-1)*9:9+(i-1)*9,:)=[pstep2_r(r,:)-pstep2_r(r,:);
            0 0 0;
            0 0 0;
            pstepm_r(r,:)-pstep2_r(r,:);
            3/2*(pstep1_r(r,1:2)-pstep2_r(r,1:2))/(tss) 0;
            0 0 0;
            pstep1_r(r,:)-pstep2_r(r,:);
            0 0 0;
            0 0 0];
    end
else
    'Choose a first SS foot'
end

%%
% pstep1_r=zeros(size(type_phase,2),3);
% pstep1_l=zeros(size(type_phase,2),3);
% j=2;
% for i=3:6:size(type_phase,2)
%     j=j+2;
%     pstep1_r(i:i+5,:)=[ones(6,1).*pstep1_(j-1,1) ones(6,1).*pstep1_(j-1,2) ones(6,1).*pstep1_(j-1,3)];
%     pstep1_l(i-2:i+5-2,:)=[ones(6,1).*pstep1_(j-2,1) ones(6,1).*pstep1_(j-2,2) ones(6,1).*pstep1_(j-2,3)];
% end
% pstep1_r(1:2,:)=[pstep1_(1,:);pstep1_(1,:)];
% pstep1_l(end-1:end,:)=[pstep1_(end,:);pstep1_(end,:)];
% pstep1_r_dt=compute_dt_type_phase(pstep1_r',discretization,nbphases,nbpointdiscret);
% pstep1_l_dt=compute_dt_type_phase(pstep1_l',discretization,nbphases,nbpointdiscret);

pstep2_r_=zeros(size(type_phase,2),3);
pstep2_l_=zeros(size(type_phase,2),3);
r=0;l=0;
for i=1:size(type_phase,2)
    if type_phase(i)==1||i==2
        if firstSS==0
            if r==l
                r=r+1;
                pstep2_r_(i,:)=pstep2_r(r,:);
            else
                l=l+1;
                pstep2_l_(i,:)=pstep2_l(l,:);
            end
        elseif firstSS==1
            if r==l
                l=l+1;
                pstep2_l_(i,:)=pstep2_l(l,:);
            else
                r=r+1;
                pstep2_r_(i,:)=pstep2_r(r,:);
            end
        end
    elseif type_phase(i)==2&&i~=2
        if firstSS==0
            if r==l
                pstep2_l_(i,:)=pstep2_l(l,:);
            else
                pstep2_r_(i,:)=pstep2_r(r,:);
            end
        elseif firstSS==1
            if r==l
                pstep2_r_(i,:)=pstep2_r(r,:);
            else
                pstep2_l_(i,:)=pstep2_l(l,:);
            end
        end
    end
end
    
pstep2_r_dt=compute_dt_type_phase(pstep2_r_',discretization,nbphases,nbpointdiscret);
pstep2_l_dt=compute_dt_type_phase(pstep2_l_',discretization,nbphases,nbpointdiscret);

%% %foot in air velocity
xvinair_r=xAvinair*param_in_air_r(:,1);
yvinair_r=yAvinair*param_in_air_r(:,2);
zvinair_r=zAvinair*param_in_air_r(:,3);
xvinair_r=xvinair_r.*any(dt_type_phase~=0,2);
yvinair_r=yvinair_r.*any(dt_type_phase~=0,2);
zvinair_r=zvinair_r.*any(dt_type_phase~=0,2);

xvinair_l=xAvinair*param_in_air_l(:,1);
yvinair_l=yAvinair*param_in_air_l(:,2);
zvinair_l=zAvinair*param_in_air_l(:,3);
xvinair_l=xvinair_l.*any(dt_type_phase~=0,2);
yvinair_l=yvinair_l.*any(dt_type_phase~=0,2);
zvinair_l=zvinair_l.*any(dt_type_phase~=0,2);

%% %foot in air acceleration
xainair_r=xAainair*param_in_air_r(:,1);
yainair_r=yAainair*param_in_air_r(:,2);
zainair_r=zAainair*param_in_air_r(:,3);
xainair_r=xainair_r.*any(dt_type_phase~=0,2);
yainair_r=yainair_r.*any(dt_type_phase~=0,2);
zainair_r=zainair_r.*any(dt_type_phase~=0,2);

xainair_l=xAainair*param_in_air_l(:,1);
yainair_l=yAainair*param_in_air_l(:,2);
zainair_l=zAainair*param_in_air_l(:,3);
xainair_l=xainair_l.*any(dt_type_phase~=0,2);
yainair_l=yainair_l.*any(dt_type_phase~=0,2);
zainair_l=zainair_l.*any(dt_type_phase~=0,2);

%% %foot in air position
% xpinair_r=xApinair*param_in_air_r(:,1)+pstep1_r_dt(:,1);
% ypinair_r=yApinair*param_in_air_r(:,2)+pstep1_r_dt(:,2);
% zpinair_r=zApinair*param_in_air_r(:,3)+pstep1_r_dt(:,3);
% xpinair_r=xpinair_r.*any(xvinair_r,2);
% ypinair_r=ypinair_r.*any(yvinair_r,2);
% zpinair_r=zpinair_r.*any(yvinair_r,2);
% xpinair_r_=xpinair_r(any(dt_type_phase~=0,2),:);
% ypinair_r_=ypinair_r(any(dt_type_phase~=0,2),:);
% zpinair_r_=zpinair_r(any(dt_type_phase~=0,2),:);
% 
% xpinair_l=xApinair*param_in_air_l(:,1)+pstep1_l_dt(:,1);
% ypinair_l=yApinair*param_in_air_l(:,2)+pstep1_l_dt(:,2);
% zpinair_l=zApinair*param_in_air_l(:,3)+pstep1_l_dt(:,3);
% xpinair_l=xpinair_l.*any(xvinair_l,2);
% ypinair_l=ypinair_l.*any(yvinair_l,2);
% zpinair_l=zpinair_l.*any(zvinair_l,2);
% xpinair_l_=xpinair_l(any(dt_type_phase~=0,2),:);
% ypinair_l_=ypinair_l(any(dt_type_phase~=0,2),:);
% zpinair_l_=zpinair_l(any(dt_type_phase~=0,2),:);

xpinair_r=xApinair*param_in_air_r(:,1)+pstep2_r_dt(:,1);
ypinair_r=yApinair*param_in_air_r(:,2)+pstep2_r_dt(:,2);
zpinair_r=zApinair*param_in_air_r(:,3)+pstep2_r_dt(:,3);
% xpinair_r=xpinair_r.*any(xvinair_r,2);
% ypinair_r=ypinair_r.*any(yvinair_r,2);
% zpinair_r=zpinair_r.*any(yvinair_r,2);
xpinair_r_=xpinair_r(any(dt_type_phase~=0,2),:);
ypinair_r_=ypinair_r(any(dt_type_phase~=0,2),:);
zpinair_r_=zpinair_r(any(dt_type_phase~=0,2),:);

xpinair_l=xApinair*param_in_air_l(:,1)+pstep2_l_dt(:,1);
ypinair_l=yApinair*param_in_air_l(:,2)+pstep2_l_dt(:,2);
zpinair_l=zApinair*param_in_air_l(:,3)+pstep2_l_dt(:,3);
% xpinair_l=xApinair*param_in_air_l(:,1);
% ypinair_l=yApinair*param_in_air_l(:,2);
% zpinair_l=zApinair*param_in_air_l(:,3);

% xpinair_l=xpinair_l.*any(xvinair_l,2);
% ypinair_l=ypinair_l.*any(yvinair_l,2);
% zpinair_l=zpinair_l.*any(zvinair_l,2);
xpinair_l_=xpinair_l(any(dt_type_phase~=0,2),:);
ypinair_l_=ypinair_l(any(dt_type_phase~=0,2),:);
zpinair_l_=zpinair_l(any(dt_type_phase~=0,2),:);


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
% for i=1:size(pstep,1)
% %     rectangle('Position',[pstep(i,1)-backtoankle,pstep(i,2)-inttoankle*mod(i,2)-exttoankle*mod(i+1,2),b,a],'EdgeColor','black','LineStyle',':','LineWidth',2)
%     rectangle('Position',[pstep(i,1)-backtoankle,pstep(i,2)-inttoankle*mod(i+1,2)-exttoankle*mod(i,2),b,a],'EdgeColor','black','LineStyle',':','LineWidth',2)
% 
% end
% rectangle('Position',[pstep(1,1)-backtoankle,-pstep(1,2)-inttoankle*mod(2,2)-exttoankle*mod(2+1,2),b,a],'EdgeColor','black','LineStyle',':','LineWidth',2)
% rectangle('Position',[pstep(size(pstep,1),1)-backtoankle,pstep(size(pstep,1),2)-(-1)^(nbstep)*0.095*2-inttoankle*mod(size(pstep,1)+1,2)-exttoankle*mod(size(pstep,1)+1+1,2),b,a],'EdgeColor','black','LineStyle',':','LineWidth',2)
XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
for i=1:length(pstep)
    plot3(XY(i,1:5),XY(i,6:10),zeros(1,5),':k','LineWidth',2)
end
hold off
%%%%%%%

open('AA_writing_in_txt.m');

% %% %%%Clear figure(9) to draw 3D%%%
% figure(9);
% clf;
% view(3);
% % set(gca,'fontsize',14,'DataAspectRatioMode','auto','PlotBoxAspectRatio',[1 1 1])
% set(gca,'fontsize',14,'DataAspectRatio',[1 1 1],'PlotBoxAspectRatio',[1 1 1])
% % axis tight
% % axis normal
% % axis('DataAspectRatioMode','auto','PlotBoxAspectRatio',[1 1 1])
% title('Interpolated ankle position')
% xlabel('x[m]')
% ylabel('y[m]')
% zlabel('z[m]')
% %%%%%%%%%%%
% %%
% figure(9)
% hold on
% % plot3(xpinair_,ypinair_,zpinair_,'+')
% plot3(xpinair_r(any(zpinair_r,2)),ypinair_r(any(zpinair_r,2)),zpinair_r(any(zpinair_r,2)),'+g','LineWidth',4)
% plot3(xpinair_l(any(zpinair_l,2)),ypinair_l(any(zpinair_l,2)),zpinair_l(any(zpinair_l,2)),'+r','LineWidth',4)
% hold off
% 
% %% %%%drawing foot steps%%%
% figure(9)
% hold on;
% XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
% for i=1:length(pstep)
%     plot3(XY(i,1:5),XY(i,6:10),zeros(1,5),':k','LineWidth',2)
% end
% hleg = legend('right ankle','left ankle','Foot edge','Location','EastOutside');
% hold off
% %%%%%%%
