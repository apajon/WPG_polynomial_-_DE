function []=ankleDSP(walking_param,save_FEM_result,save_ankle_result)
    clc
    % fet1 ______ fet3
    %     |      |
    %     |______|
    % fet2        fet4

    %%
    load(save_FEM_result);

    %%
% % [xA1 yA1 zA1 xA2 yA2 zA2 xA3 yA3 zA3 xA4 yA4 zA4]=textread('test_CIR.txt', '%f %f %f %f %f %f %f %f %f %f %f %f', 'headerlines', 1);
% fet=load('00_foot_edge_top.mat');
% tpp=load('00_theta_phi_psi.mat');

%% 
% backtoankle=0.098; %from back to ankle of foot
% fronttoankle=0.128; %from  front to ankle of foot
% exttoankle=0.076; %from exterior to ankle of foot
% inttoankle=0.054; %from interior to ankle of foot
% xpankle=1.257728354176543; %x coordinate of ankle position
% ypankle=-0.045000000000611; %y coordinate of ankle position
% rightorleft=1;%+1 for right foot and -1 for left foot
% ha=0.08;
% walking_param.e=0.021;%real sole width
% walking_param.e_=0.011;%max sole width

%%
Lfooty = walking_param.exttoankle+walking_param.inttoankle;
Lfootx = walking_param.backtoankle+walking_param.fronttoankle;

%% 
% traslx=xpankle-backtoankle;
% trasly=ypankle-rightorleft*(Lfooty/2-inttoankle);
% traslz=e;
traslx=zeros(size(walking_param.pstep,1),1);
trasly=zeros(size(walking_param.pstep,1),1);
traslz=ones(size(walking_param.pstep,1),1)*(walking_param.e-walking_param.e_);

for i=1:size(walking_param.pstep,1)
    rightorleft=(-1)^(walking_param.firstSS+i+1);
    
    traslx(i)=walking_param.pstep(i,1)-walking_param.backtoankle;
    trasly(i)=walking_param.pstep(i,2)-rightorleft*(Lfooty/2-walking_param.inttoankle);
end
%%
traslx_r=zeros(walking_param.nbpointdiscret,1);
trasly_r=zeros(walking_param.nbpointdiscret,1);
traslz_r=zeros(walking_param.nbpointdiscret,1);
traslx_l=zeros(walking_param.nbpointdiscret,1);
trasly_l=zeros(walking_param.nbpointdiscret,1);
traslz_l=zeros(walking_param.nbpointdiscret,1);
for i=1:size(walking_param.pstep,1)
    zeros_=zeros(walking_param.nbpointdiscret,1);
    if i == 1
        zeros_(1:sum(walking_param.discretization(1))+1)=1;
    elseif i == 2
        zeros_(1:sum(walking_param.discretization(1:3))+1)=1;
    elseif i==walking_param.nbpankle-1
        zeros_(sum(walking_param.discretization(1:(i-3)*3+2))+1+1:sum(walking_param.discretization(1:(i-3)*3+3+2))+1)=1;
    elseif i==walking_param.nbpankle
        zeros_(sum(walking_param.discretization(1:(i-3)*3+1))+1+1:sum(walking_param.discretization(1:(i-3)*3+2))+1)=1;
    else
        zeros_(sum(walking_param.discretization(1:(i-3)*3+2))+1+1:sum(walking_param.discretization(1:(i-3)*3+3+3))+1)=1;
    end
    
    rightorleft=(-1)^(walking_param.firstSS+i+1);
    if rightorleft==+1
        traslx_r=traslx_r+(walking_param.pstep(i,1)-walking_param.backtoankle)*zeros_;
        trasly_r=trasly_r+(walking_param.pstep(i,2)-rightorleft*(Lfooty/2-walking_param.inttoankle))*zeros_;
        traslz_r=traslz_r+(-walking_param.e+walking_param.e_)*zeros_;
    else
        traslx_l=traslx_l+(walking_param.pstep(i,1)-walking_param.backtoankle)*zeros_;
        trasly_l=trasly_l+(walking_param.pstep(i,2)-rightorleft*(Lfooty/2-walking_param.inttoankle))*zeros_;
        traslz_l=traslz_l+(-walking_param.e+walking_param.e_)*zeros_;
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
punderfoot_r=[ones(walking_param.nbpointdiscret,1)*walking_param.backtoankle ones(walking_param.nbpointdiscret,1)*(Lfooty/2-walking_param.inttoankle) ones(walking_param.nbpointdiscret,1)*0];
punderfoot_l=[ones(walking_param.nbpointdiscret,1)*walking_param.backtoankle -ones(walking_param.nbpointdiscret,1)*(Lfooty/2-walking_param.inttoankle) ones(walking_param.nbpointdiscret,1)*0];

pankleground_r=dir24_r.*[punderfoot_r(:,1) punderfoot_r(:,1) punderfoot_r(:,1)]+dir21_r.*[punderfoot_r(:,2) punderfoot_r(:,2) punderfoot_r(:,2)]+dirnormal124_r.*[punderfoot_r(:,3) punderfoot_r(:,3) punderfoot_r(:,3)]+(fet1_r_+fet2_r_)/2;
pankleground_l=dir24_l.*[punderfoot_l(:,1) punderfoot_l(:,1) punderfoot_l(:,1)]+dir21_l.*[punderfoot_l(:,2) punderfoot_l(:,2) punderfoot_l(:,2)]+dirnormal124_l.*[punderfoot_l(:,3) punderfoot_l(:,3) punderfoot_l(:,3)]+(fet1_l_+fet2_l_)/2;

pankle_r_=pankleground_r+dirnormal124_r*(walking_param.ha-walking_param.e_);
pankle_l_=pankleground_l+dirnormal124_l*(walking_param.ha-walking_param.e_);


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
plot3(walking_param.pstep(:,1),walking_param.pstep(:,2),ones(walking_param.nbpankle,1)*walking_param.ha,'o')
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
for i=1:size(Z_r,1)
% for i=1
    figure(10)
    clf
%     view(3)
    view(2)
    grid on
    set(gca,'fontsize',14,'DataAspectRatio',[1 1 1],'PlotBoxAspectRatio',[1 1 1])
    hold on
    plot3(X_r(i,:),Y_r(i,:),Z_r(i,:))
    plot3(pankle_r(i,1),pankle_r(i,2),pankle_r(i,3),'*')
    plot3(X_l(i,:),Y_l(i,:),Z_l(i,:))
    plot3(pankle_l(i,1),pankle_l(i,2),pankle_l(i,3),'*')
    plot3(0,0,0,'o')
    hold off
end
%%
    
%     %%
    save(save_ankle_result,'xponfloor_r','yponfloor_r','zponfloor_r','xponfloor_l','yponfloor_l','zponfloor_l','rpsi_dt_r','rphi_dt_r','rtheta_dt_r','rpsi_dt_l','rphi_dt_l','rtheta_dt_l');
    walking_param.xpankle_r=xponfloor_r;
    walking_param.ypankle_r=yponfloor_r;
    walking_param.zpankle_r=zponfloor_r;
    walking_param.xpankle_l=xponfloor_l;
    walking_param.ypankle_l=yponfloor_l;
    walking_param.zpankle_l=zponfloor_l;
    walking_param.rpsi_r=rpsi_dt_r;
    walking_param.rphi_r=rphi_dt_r;
    walking_param.rtheta_l=rtheta_dt_r;
    walking_param.rphi_l=rpsi_dt_l;
    walking_param.rpsi_l=rphi_dt_l;
    walking_param.rtheta_l=rtheta_dt_l;
end