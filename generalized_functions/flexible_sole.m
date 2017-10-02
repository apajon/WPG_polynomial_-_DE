function []=flexible_sole(walking_param,footstepwanted,Young,Poisson,filename)

% format long
% close all
% fclose('all')

% %for windows
addpath .\Simulation_3D
addpath .\Simulation_3D\input
addpath .\Simulation_3D\results
addpath .\Simulation_3D\FEM
    
% %for linux
% addpath ./Simulation_3D
% addpath ./Simulation_3D\input
% addpath ./Simulation_3D\results
% addpath ./Simulation_3D\FEM

load('trajectories.mat')

% clear COMx COMy Fdes Fx Fy Fz ZMPx ZMPy Zdes azimuth coor cost elevation fname friction i l0 p_mid pname r rightorleft sole soleini spl t u0 xpankle ypankle

%%%writing ZMP and COM trajectories in 'zmp&com.txt' file%%%%
% trajectories=[xpzmp(1)*ones(137,1) ypzmp(1)*ones(137,1) xpcom(1)*ones(137,1) ypcom(1)*ones(137,1) 0*ones(137,1) 0*ones(137,1)];
% trajectories=[];

time=(0:sum(walking_param.discretization))/walking_param.frequency;

discretization_=walking_param.discretization(any(walking_param.type_phase==0,1));
%% %zmp1 force
dt1=compute_coeff_dt1_matrix(walking_param.discretization,walking_param.frequency,0.5);
dt1=dt1(any(walking_param.dt_type_phase==0,2),:);
[Ac]=compute_repartition_matrix(walking_param.tpassage,0.5);
zfzmp1=dt1*Ac;

%%
fet1_r=[];
fet2_r=[];
fet3_r=[];
fet4_r=[];
tpp_r=[];

fet1_l=[];
fet2_l=[];
fet3_l=[];
fet4_l=[];
tpp_l=[];
%%
for j=footstepwanted;
%     for j = 1:nbpankle
%     for j = 13
    clear COMx COMy Fdes Fx Fy Fz ZMPx ZMPy Zdes azimuth coor cost elevation fname friction l0 p_mid pname r rightorleft sole soleini spl t u0 xpankle ypankle
    tic

    %% %choose the foot step
    foot_step_wanted=j
    % WARNING Bug with #last and #before last foot steps

    %% %choosen foot step coordinate
    foot_step_coor=walking_param.pstep(foot_step_wanted,:);

    %% %compute zmp under one foot
    [time_ pzmp pcom fzmp]=zmp_under_foot(foot_step_wanted,walking_param.nbpankle,time,walking_param.xpzmp,walking_param.ypzmp,walking_param.xpzmp1,walking_param.ypzmp1,walking_param.xpzmp2,walking_param.ypzmp2,walking_param.xpcom,walking_param.ypcom,walking_param.z*ones(size(walking_param.xpcom)),zfzmp1,xAfcom,xBfcom,yAfcom,yBfcom,walking_param.psa_abcd,walking_param.discretization,discretization_,walking_param.mg);

    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                       Sole FEM                               %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fname = 'semelle1.msh';
    pname = 'Simulation_3D/input/semelle1 L=0.23, l=0.13, e=0.021 m new centre/';

    %%% foot size %%%
    l=0.13;
    L=0.23;
    e=0.021;
    sole = soleFEM_newStiff(pname,fname,l,L,e);
    coorini = sole.coor;
    friction = 0.65;
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                               ZMP                            %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Linux
    % [t, ZMPx, ZMPy, COMx, COMy, Fx, Fy, Fz] = textread('/media/63DFD35C3FD8A3D0/Giappone/Code/sim3d/Simulation 3D - Desired ZMP - Position and Force - 3 Angles - meters - Linux/trajectory/exemple_trajectoire.txt', '%f %f %f %f %f %f %f %f', 'headerlines', 1);
    % Windows
%         if exist('Simulation_3D/trajectory/exemple_trajectoire.txt','file')~=0
%             % Linux
%             [t, ZMPx, ZMPy, COMx, COMy, Fx, Fy, Fz] = textread('Simulation_3D/trajectory/exemple_trajectoire.txt', '%f %f %f %f %f %f %f %f', 'headerlines', 1);
%         elseif exist('Simulation_3D\trajectory\exemple_trajectoire.txt','file')~=0
%             % Windows
%             [t, ZMPx, ZMPy, COMx, COMy, Fx, Fy, Fz] = textread('Simulation_3D\trajectory\exemple_trajectoire.txt', '%f %f %f %f %f %f %f %f', 'headerlines', 1);
%         end
%         [Zdes,Fdes] = changeRefxxx(ZMPx, ZMPy, Fx, Fy, Fz);
    % Fdes(2,:)=0;
    % Zdes(2,:)=0;
    t=time_;
    ZMPx=pzmp(:,1);
    ZMPy=pzmp(:,2);
    COMx=pcom(:,1);
    COMy=pcom(:,2);
    Fx=fzmp(:,1);
    Fy=fzmp(:,2);
    Fz=fzmp(:,3);
    % backtoankle=0.098; %from back to ankle of foot
    % fronttoankle=0.128; %from  front to ankle of foot
    % exttoankle=0.076; %from exterior to ankle of foot
    % inttoankle=0.054; %from interior to ankle of foot
    % xpankle=1.257728354176543; %x coordinate of ankle position
    % ypankle=-0.045000000000611; %y coordinate of ankle position
    xpankle=foot_step_coor(:,1);
    ypankle=foot_step_coor(:,2);
    %     rightorleft=1;%+1 for right foot and -1 for left foot
    rightorleft=(-1)^(foot_step_wanted+1+walking_param.firstSS);%+1 for right foot and -1 for left foot

    [Zdes,Fdes] = changeRef(ZMPx, ZMPy, Fx, Fy, Fz,walking_param.backtoankle,walking_param.fronttoankle,walking_param.exttoankle,walking_param.inttoankle,xpankle,ypankle,rightorleft);

%         Zdes(:,end-10:end)=[];
%         Zdes(:,1:10)=[];
    Zdes = Zdes(:,1:1:length(Zdes));
    % Zdes(:,1:5)=[];
    % Zdes(:,end-1:end)=[];

%         Fdes(:,end-10:end)=[];
%         Fdes(:,1:10)=[];
    Fdes = Fdes(:,1:1:length(Fdes));
    % Fdes(:,1:5)=[];
    % Fdes(:,end-1:end)=[];
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                           B-spline                           %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    coor = zeros(sole.nTot,3);
    coor(:,1)= sole.coor(:,1) - sole.trasl(1);
    coor(:,2) = sole.coor(:,2) - sole.trasl(2);
    coor(:,3) = sole.coor(:,3) - sole.trasl(3) - sole.zpoles;
    azimuth = zeros(1,size(coor,1));
    elevation = zeros(1,size(coor,1));
    r = zeros(1,size(coor,1));
    for i=1:size(sole.coor,1);
        [azimuth(1,i),elevation(1,i),r(1,i)] = cart2sph(-coor(i,3),coor(i,2),coor(i,1));
    end
    spline_res = -pi/2:pi/10:pi/2;
    spl = splineBasis([azimuth;elevation;r], spline_res, spline_res);
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                        Shape Optimization                    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_mid = ones(length(spline_res)+2,length(spline_res)+2);
    % a = load('polynomeroughsmooth.mat');
    % % % % %  a = load('polynome24-03-2015-1st-part.mat');
    % % % % % % a = load('polynome24-03-2015-2nd-part.mat');
    % p_mid = a.p_mid;
    % % % p_mid = p_mid - 0.05;
%         p_mid_Old = p_mid;
    % % 
    %  b = load('polynome23-03-2015.mat');
    % % p_mid2 = b.p_mid;
    %  p_mid = b.p_mid;
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                           Remesh                             %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % coornew = deformation(size(sole.coor),p_mid,spl);
    % % %coornew = deformation(size(sole.coor),p_mid2-(ones(11,11)-p_mid),spl);
    % coornew(:,1)= coornew(:,1) + sole.trasl(1);
    % coornew(:,2) = coornew(:,2) + sole.trasl(2);
    % coornew(:,3) = coornew(:,3) + sole.trasl(3);
    % coornewtmp = remesh(sole,pname,coornew);
    % [D,I] = pdist2(coornew,coornewtmp,'euclidean','Smallest',1);       
    % sole.coor(I,:) = coornewtmp; 
    % writemshfile(sole,pname)
    % stressVM0 = zeros(sole.nTot,1);
    % plotsole(1,sole.elements_surf,sole.coor,stressVM0,[],[],[],[]);
    % 
    % 
    % coor = zeros(sole.nTot,3);
    % coor(:,1) = sole.coor(:,1) - sole.trasl(1);
    % coor(:,2) = sole.coor(:,2) - sole.trasl(2);
    % coor(:,3) = sole.coor(:,3) - sole.trasl(3);
    % azimuth = zeros(1,size(coor,1));
    % elevation = zeros(1,size(coor,1));
    % r = zeros(1,size(coor,1));
    % for i=1:size(sole.coor,1);
    %     [azimuth(1,i),elevation(1,i),r(1,i)] = cart2sph(-coor(i,3),coor(i,2),coor(i,1));
    % end
    % spl = splineBasis([azimuth;elevation;r], spline_res, spline_res);
    % stressVM0 = zeros(sole.nTot,1);
    % plotsole(2,sole.elements_surf,sole.coor,stressVM0,[],[],[],[]);
    %  
    % p_mid = ones(length(spline_res)+2,length(spline_res)+2);

    % Poisson ratio caucciu = 0.5
    % Young Modulus rubber = 10^7 Pa; rubber = 0.01-0.1 * 10^9 Pa
    % Elastomer
    % Butyl Rubber 0.001-0.002 GPa
    % Sylicon Elastomers 0.005-0.02 GPa
    % Neoprene (CR) 0.0007-0.002 GPa
    % Neoprene (CR)
%         Young = 700001;
%         Young = 120000;
%         Poisson = 0.3;
    sole.setMaterial(Young,Poisson);

    stressVM0 = zeros(sole.nTot,1);
%         plotsole(1,sole.elements_surf,sole.coor,stressVM0,[],[],[],[]);
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                           Optimization                       %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         u0 = (zeros(length(spline_res)+2,length(spline_res)+2)+1.4)+(ones(length(spline_res)+2,length(spline_res)+2)-p_mid_Old);
%         l0 = (zeros(length(spline_res)+2,length(spline_res)+2)+0.4)+(ones(length(spline_res)+2,length(spline_res)+2)-p_mid_Old);
    %u0 = (zeros(length(spline_res)+2,length(spline_res)+2)+1.3);
    %l0 = (zeros(length(spline_res)+2,length(spline_res)+2)+0.6);
    u0 = (zeros(length(spline_res),length(spline_res))+1.5);
    l0 = (zeros(length(spline_res),length(spline_res))+0.4);

    %optimization(p_mid,sole,soleini,friction,Fdes,Zdes,spl)
    %fmincon stopped because it exceeded the function evaluation limit,
    %options.MaxFunEvals = 3000 (the default value).
    %options = optimset('Display','iter','Algorithm','Interior-Point','MaxFunEvals',3000000000000000000000000000000000,'FinDiffType','central');
    options = optimset('Display','iter','Algorithm','Interior-Point','MaxFunEvals',3000000000000000000000000000000000,'TolX',1e-4,'TolFun',1e-4);
    %options = optimset('Display','iter','Algorithm','Interior-Point','SubproblemAlgorithm','cg');
    % options = optimset('Display','iter','Algorithm','sqp','TolCon',1e-10);
    % options = optimset('Display','iter','Algorithm','active-set');
    %options = optimset('Display','iter','Algorithm','active-set','DiffMinChange',1e-8,'RelLineSrchBnd',0.03,'RelLineSrchBndDuration',50);
    % options = optimset('Display','iter','Algorithm','active-set','DiffMinChange',1e-8,'RelLineSrchBnd',0.03);
    %options = optimset('Display','iter','Algorithm','active-set','RelLineSrchBnd',3,'RelLineSrchBndDuration',2);
    %options = optimset('Display','iter','Algorithm','interior-point','RelLineSrchBnd',0.03);
    vol = volini(sole);
    % costFunc(p_mid,sole,friction,Fdes,Zdes,spl)
%         costFunc(p_mid,sole,friction,Fdes,Zdes,spl);
    costFunc(p_mid,sole,friction,Fdes,Zdes,spl,coorini);
    % X = fmincon(@(p_mid)costFunc(p_mid,sole,friction,Fdes,Zdes,spl),p_mid,[],[],[],[],l0,u0,@(p_mid)fminconstr_detJ(p_mid,sole,spl,vol),options);
    %X = fmincon(@(p_mid)costFunc(p_mid,sole,soleini,friction,Fdes,Zdes,spl),p_mid,[],[],[],[],l0,u0,[],options);

    % fseminf 
    %[X,fval,exitflag] = fmincon(@(p_mid)costFunc(p_mid,sole,soleini,friction,Fdes,Zdes,spl),p_mid,[],[],[],[],l0,u0,[],options);
    %psoptions = psoptimset('Display','iter');
    %X = patternsearch(func,p_mid,[],[],[],[],l0,u0,[],psoptions);
    % opts = optimset('Display','iter');
    % opts.Display = 'iter';
    % opts.TolX = 1.e-12;
    % opts.TolFun = 1.e-12;
    % opts.MaxFunEvals = 100;
    % X=fminsearchcon(@(p_mid)costFunc(p_mid,sole,soleini,friction,Fdes,Zdes,spl),p_mid,l0,u0,[],[],@(p_mid)fminconstr_detJ(p_mid,soleini,spl),opts);
    % X=fminsearch(@(p_mid)costFunc(p_mid,sole,soleini,friction,Fdes,Zdes,spl),p_mid,opts);
%%
    cost=load('Simulation_3D/results/trajectory.mat');

%         cost.tpp(1:5,:)
%         cost.fet1(1:5,:)
    if rightorleft==+1
        fet1_r=[fet1_r;cost.fet1;zeros(walking_param.discretization(2),3)];
        fet2_r=[fet2_r;cost.fet2;zeros(walking_param.discretization(2),3)];
        fet3_r=[fet3_r;cost.fet3;zeros(walking_param.discretization(2),3)];
        fet4_r=[fet4_r;cost.fet4;zeros(walking_param.discretization(2),3)];
        tpp_r=[tpp_r;cost.tpp;zeros(walking_param.discretization(2),3)];
    else
        fet1_l=[fet1_l;cost.fet1;zeros(walking_param.discretization(2),3)];
        fet2_l=[fet2_l;cost.fet2;zeros(walking_param.discretization(2),3)];
        fet3_l=[fet3_l;cost.fet3;zeros(walking_param.discretization(2),3)];
        fet4_l=[fet4_l;cost.fet4;zeros(walking_param.discretization(2),3)];
        tpp_l=[tpp_l;cost.tpp;zeros(walking_param.discretization(2),3)];
    end

    toc

%         close 1 2 4
end
%%
%     tpp=tpp_r(1:end-discretization(2));
%%
fet1_r_=fet1_r(1:end-walking_param.discretization(2),:);
fet2_r_=fet2_r(1:end-walking_param.discretization(2),:);
fet3_r_=fet3_r(1:end-walking_param.discretization(2),:);
fet4_r_=fet4_r(1:end-walking_param.discretization(2),:);
tpp_r_=tpp_r(1:end-walking_param.discretization(2),:);

fet1_l_=fet1_l(1:end-walking_param.discretization(2),:);
fet2_l_=fet2_l(1:end-walking_param.discretization(2),:);
fet3_l_=fet3_l(1:end-walking_param.discretization(2),:);
fet4_l_=fet4_l(1:end-walking_param.discretization(2),:);
tpp_l_=tpp_l(1:end-walking_param.discretization(2),:);
%%
save(filename,'fet1_r_','fet2_r_','fet3_r_','fet4_r_','tpp_r_','fet1_l_','fet2_l_','fet3_l_','fet4_l_','tpp_l_')
end