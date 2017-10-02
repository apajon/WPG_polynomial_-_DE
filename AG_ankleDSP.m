clc
% fet1 ______ fet3
%     |      |
%     |______|
% fet2        fet4

%%
% % [xA1 yA1 zA1 xA2 yA2 zA2 xA3 yA3 zA3 xA4 yA4 zA4]=textread('test_CIR.txt', '%f %f %f %f %f %f %f %f %f %f %f %f', 'headerlines', 1);
% fet=load('00_foot_edge_top.mat');
% tpp=load('00_theta_phi_psi.mat');
%%
%  load('result_FEM_06_15_oscillation_tss800_tds400_320000.mat')
 fet1_r_=[any(fet2_r_,2)*0 any(fet1_r_,2)*0.0650 any(fet1_r_,2)*0.021];
 fet2_r_=[any(fet2_r_,2)*0 any(fet2_r_,2)*(-0.0650) any(fet2_r_,2)*0.021];
 fet3_r_=[any(fet3_r_,2)*0.23 any(fet3_r_,2)*0.0650 any(fet3_r_,2)*0.021];
 fet4_r_=[any(fet4_r_,2)*0.23 any(fet4_r_,2)*(-0.0650) any(fet4_r_,2)*0.021];
 tpp_r_=[zeros(size(tpp_r_,1),3)];
 
 fet1_l_=[any(fet2_l_,2)*0 any(fet1_l_,2)*0.0650 any(fet1_l_,2)*0.021];
 fet2_l_=[any(fet2_l_,2)*0 any(fet2_l_,2)*(-0.0650) any(fet2_l_,2)*0.021];
 fet3_l_=[any(fet3_l_,2)*0.23 any(fet3_l_,2)*0.0650 any(fet3_l_,2)*0.021];
 fet4_l_=[any(fet4_l_,2)*0.23 any(fet4_l_,2)*(-0.0650) any(fet4_l_,2)*0.021];
 tpp_l_=[zeros(size(tpp_l_,1),3)];

%  fet1_r_=[zeros(size(xpzmp,1),1) ones(size(xpzmp,1),1)*0.0650 ones(size(xpzmp,1),1)*0.021];
%  fet2_r_=[zeros(size(xpzmp,1),1) ones(size(xpzmp,1),1)*(-0.0650) ones(size(xpzmp,1),1)*0.021];
%  fet3_r_=[ones(size(xpzmp,1),1)*0.23 ones(size(xpzmp,1),1)*0.0650 ones(size(xpzmp,1),1)*0.021];
%  fet4_r_=[ones(size(xpzmp,1),1)*0.23 ones(size(xpzmp,1),1)*(-0.0650) ones(size(xpzmp,1),1)*0.021];
%  tpp_r_=[zeros(size(xpzmp,1),3)];
%  
%  fet1_l_=[zeros(size(xpzmp,1),1) ones(size(xpzmp,1),1)*0.0650 ones(size(xpzmp,1),1)*0.021];
%  fet2_l_=[zeros(size(xpzmp,1),1) ones(size(xpzmp,1),1)*(-0.0650) ones(size(xpzmp,1),1)*0.021];
%  fet3_l_=[ones(size(xpzmp,1),1)*0.23 ones(size(xpzmp,1),1)*0.0650 ones(size(xpzmp,1),1)*0.021];
%  fet4_l_=[ones(size(xpzmp,1),1)*0.23 ones(size(xpzmp,1),1)*(-0.0650) ones(size(xpzmp,1),1)*0.021];
%  tpp_l_=[zeros(size(xpzmp,1),3)];
%% 
% backtoankle=0.098; %from back to ankle of foot
% fronttoankle=0.128; %from  front to ankle of foot
% exttoankle=0.076; %from exterior to ankle of foot
% inttoankle=0.054; %from interior to ankle of foot
% xpankle=1.257728354176543; %x coordinate of ankle position
% ypankle=-0.045000000000611; %y coordinate of ankle position
% rightorleft=1;%+1 for right foot and -1 for left foot
% ha=0.08;
% e=0.021;%real sole width
% e_=0.011;%max sole width

%%
Lfooty = exttoankle+inttoankle;
Lfootx = backtoankle+fronttoankle;

%% 
% traslx=xpankle-backtoankle;
% trasly=ypankle-rightorleft*(Lfooty/2-inttoankle);
% traslz=e;

% traslx=zeros(size(pstep,1),1);
% trasly=zeros(size(pstep,1),1);
% traslz=ones(size(pstep,1),1)*(e-e_);

for i=1:size(pstep,1)
    rightorleft=(-1)^(firstSS+i+1);
    
%     traslx(i)=pstep(i,1)-backtoankle;
%     trasly(i)=pstep(i,2)-rightorleft*(Lfooty/2-inttoankle);
end
%%
traslx_r=zeros(nbpointdiscret,1);
trasly_r=zeros(nbpointdiscret,1);
traslz_r=zeros(nbpointdiscret,1);
traslx_l=zeros(nbpointdiscret,1);
trasly_l=zeros(nbpointdiscret,1);
traslz_l=zeros(nbpointdiscret,1);
for i=1:size(pstep,1)
    zeros_=zeros(nbpointdiscret,1);
    if i == 1
        zeros_(1:sum(discretization(1))+1)=1;
    elseif i == 2
        zeros_(1:sum(discretization(1:3))+1)=1;
    elseif i==nbpankle-1
        zeros_(sum(discretization(1:(i-3)*3+2))+1+1:sum(discretization(1:(i-3)*3+3+2))+1)=1;
    elseif i==nbpankle
        zeros_(sum(discretization(1:(i-3)*3+1))+1+1:sum(discretization(1:(i-3)*3+2))+1)=1;
    else
        zeros_(sum(discretization(1:(i-3)*3+2))+1+1:sum(discretization(1:(i-3)*3+3+3))+1)=1;
    end
    
    theta_=psi(1:end-1);
    theta_=theta_(any(type_phase==0,1));
    if i~=nbpankle
            theta=theta_(i);
    else
        theta=psi(end);
    end
    mrot=[cos(theta) -sin(theta);sin(theta) cos(theta)];
%     pzmp=([pzmp(:,1)-foot_step_coor(1) pzmp(:,2)-foot_step_coor(2)])*mrot;
%      pzmp(:,1)=pzmp(:,1)+foot_step_coor(1);
%     pzmp(:,2)=pzmp(:,2)+foot_step_coor(2);
%     pcom(:,1:2)=([pcom(:,1)-foot_step_coor(1) pcom(:,2)-foot_step_coor(2)])*mrot;
%     pcom(:,1)=pcom(:,1)+foot_step_coor(1);
%     pcom(:,2)=pcom(:,2)+foot_step_coor(2);
%     fzmp(:,1:2)=fzmp(:,1:2)*mrot;
% orig_foot=[-backtoankle -(Lfooty/2-inttoankle)]*inv(mrot);
        
    rightorleft=(-1)^(firstSS+i+1);
    orig_foot=[-backtoankle -rightorleft*(Lfooty/2-inttoankle)]*inv(mrot);
    if rightorleft==+1
%         orig_foot=[-backtoankle -(Lfooty/2-inttoankle)]*inv(mrot);
        traslx_r=traslx_r+(pstep(i,1)+orig_foot(1))*zeros_;
        trasly_r=trasly_r+(pstep(i,2)+orig_foot(2))*zeros_;
%         traslx_r=traslx_r+(pstep(i,1)-backtoankle)*zeros_;
%         trasly_r=trasly_r+(pstep(i,2)-rightorleft*(Lfooty/2-inttoankle))*zeros_;
%         traslz_r=traslz_r+(-e+e_)*zeros_;
    else
%         orig_foot=[-backtoankle +(Lfooty/2-inttoankle)]*inv(mrot);
        traslx_l=traslx_l+(pstep(i,1)+orig_foot(1))*zeros_;
        trasly_l=trasly_l+(pstep(i,2)+orig_foot(2))*zeros_;
%         traslx_l=traslx_l+(pstep(i,1)-backtoankle)*zeros_;
%         trasly_l=trasly_l+(pstep(i,2)-rightorleft*(Lfooty/2-inttoankle))*zeros_;
%         traslz_l=traslz_l+(-e+e_)*zeros_;
    end
end
%%
dir21_r=(fet1_r_-fet2_r_);
dir24_r=(fet4_r_-fet2_r_);
dir21_l=(fet1_l_-fet2_l_);
dir24_l=(fet4_l_-fet2_l_);
for i=1:size(dir21_r,1)
    dir21_r(i,:)=dir21_r(i,:)/norm(dir21_r(i,:));
    dir21_r(isnan(dir21_r))=0;
    dir24_r(i,:)=dir24_r(i,:)/norm(dir24_r(i,:));
    dir24_r(isnan(dir24_r))=0;
    
    dir21_l(i,:)=dir21_l(i,:)/norm(dir21_l(i,:));
    dir21_l(isnan(dir21_l))=0;
    dir24_l(i,:)=dir24_l(i,:)/norm(dir24_l(i,:));
    dir24_l(isnan(dir24_l))=0;
end

dirnormal124_r=cross(dir24_r,dir21_r);
dirnormal124_l=cross(dir24_l,dir21_l);
%%
punderfoot_r=[ones(nbpointdiscret,1)*backtoankle ones(nbpointdiscret,1)*(Lfooty/2-inttoankle) ones(nbpointdiscret,1)*0];
punderfoot_l=[ones(nbpointdiscret,1)*backtoankle -ones(nbpointdiscret,1)*(Lfooty/2-inttoankle) ones(nbpointdiscret,1)*0];

pankleground_r=dir24_r.*[punderfoot_r(:,1) punderfoot_r(:,1) punderfoot_r(:,1)]+dir21_r.*[punderfoot_r(:,2) punderfoot_r(:,2) punderfoot_r(:,2)]+dirnormal124_r.*[punderfoot_r(:,3) punderfoot_r(:,3) punderfoot_r(:,3)]+(fet1_r_+fet2_r_)/2;
pankleground_l=dir24_l.*[punderfoot_l(:,1) punderfoot_l(:,1) punderfoot_l(:,1)]+dir21_l.*[punderfoot_l(:,2) punderfoot_l(:,2) punderfoot_l(:,2)]+dirnormal124_l.*[punderfoot_l(:,3) punderfoot_l(:,3) punderfoot_l(:,3)]+(fet1_l_+fet2_l_)/2;

pankle_r_=pankleground_r+dirnormal124_r*(ha-e_);
pankle_l_=pankleground_l+dirnormal124_l*(ha-e_);


pankle_r=pankle_r_+[traslx_r trasly_r traslz_r];
pankle_l=pankle_l_+[traslx_l trasly_l traslz_l];

xponfloor_r=pankle_r(:,1); yponfloor_r=pankle_r(:,2); zponfloor_r=pankle_r(:,3);
xponfloor_l=pankle_l(:,1); yponfloor_l=pankle_l(:,2); zponfloor_l=pankle_l(:,3);
%%
rpsi_dt_r=tpp_r_(:,3);
rphi_dt_r=tpp_r_(:,2);
rtheta_dt_r=tpp_r_(:,1);
rpsi_dt_l=tpp_l_(:,3);
rphi_dt_l=tpp_l_(:,2);
rtheta_dt_l=tpp_l_(:,1);

%%
figure(1)
clf
view(3)
grid on
set(gca,'fontsize',14,'DataAspectRatio',[1 1 1],'PlotBoxAspectRatio',[1 1 1])
hold on
% plot3(xpankle,ypankle,0,'o')
% plot3(xpankle,ypankle,e,'o')
% plot3(xpankle,ypankle,e+ha,'o')
plot3(pstep(:,1),pstep(:,2),ones(nbpankle,1)*(ha+(e-e_)),'o')
plot3(xponfloor_r,yponfloor_r,zponfloor_r,'*g')
plot3(xponfloor_l,yponfloor_l,zponfloor_l,'*r')
hold off
% 
% figure(2)
% clf
% plot(pankle_r(:,1),'-*g')
% plot(pankle_l(:,1),'-*r')
% 
% figure(3)
% clf
% plot(pankle_r(:,2),'-*g')
% plot(pankle_l(:,2),'-*r')
% 
% figure(4)
% clf
% plot(pankle_r(:,3),'-*g')
% plot(pankle_l(:,3),'-*r')

% %%
% X=[fet1_r_(:,1) fet2_r_(:,1) fet4_r_(:,1) fet3_r_(:,1) fet1_r_(:,1)];
% Y=[fet1_r_(:,2) fet2_r_(:,2) fet4_r_(:,2) fet3_r_(:,2) fet1_r_(:,2)];
% Z=[fet1_r_(:,3) fet2_r_(:,3) fet4_r_(:,3) fet3_r_(:,3) fet1_r_(:,3)];
% for i=1:size(Z,1)
%     figure(10)
%     clf
%     view(3)
%     grid on
%     set(gca,'fontsize',14,'DataAspectRatio',[1 1 1],'PlotBoxAspectRatio',[1 1 1])
%     hold on
%     plot3(X(i,:),Y(i,:),Z(i,:))
%     plot3(pankle_r_(i,1),pankle_r_(i,2),pankle_r_(i,3),'*')
%     hold off
% end
%%
X_r=[fet1_r_(:,1) fet2_r_(:,1) fet4_r_(:,1) fet3_r_(:,1) fet1_r_(:,1)]+[traslx_r traslx_r traslx_r traslx_r traslx_r];
Y_r=[fet1_r_(:,2) fet2_r_(:,2) fet4_r_(:,2) fet3_r_(:,2) fet1_r_(:,2)]+[trasly_r trasly_r trasly_r trasly_r trasly_r];
Z_r=[fet1_r_(:,3) fet2_r_(:,3) fet4_r_(:,3) fet3_r_(:,3) fet1_r_(:,3)]+[traslz_r traslz_r traslz_r traslz_r traslz_r];

X_l=[fet1_l_(:,1) fet2_l_(:,1) fet4_l_(:,1) fet3_l_(:,1) fet1_l_(:,1)]+[traslx_l traslx_l traslx_l traslx_l traslx_l];
Y_l=[fet1_l_(:,2) fet2_l_(:,2) fet4_l_(:,2) fet3_l_(:,2) fet1_l_(:,2)]+[trasly_l trasly_l trasly_l trasly_l trasly_l];
Z_l=[fet1_l_(:,3) fet2_l_(:,3) fet4_l_(:,3) fet3_l_(:,3) fet1_l_(:,3)]+[traslz_l traslz_l traslz_l traslz_l traslz_l];
% for i=1:size(Z_r,1)
 %%
open('AH_drawing3D.m');