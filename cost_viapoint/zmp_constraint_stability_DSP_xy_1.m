function [Acons Bcons] = zmp_constraint_stability_DSP_xy_1(xApzmp,xBpzmp,yApzmp,yBpzmp,xApankle,xBpankle,yApankle,yBpankle,discretization,backtoankle,fronttoankle,exttoankle,inttoankle,sole_margin,theta,type_phase,nbparamABCD,nbparamBb,firstSS)

% [xApankle xBpankle]=torque_ankle_positions_DSP_12(pankinit2(1),pankfin1(1),discretization);
% [yApankle yBpankle]=torque_ankle_positions_DSP_12(pankinit2(2),pankfin1(2),discretization);
% yBpankle(1:discretization(1)+1)=-yBpankle(1:discretization(1)+1);
% xBpankle(1:discretization(1)+1)=pankinit1(1);
% yBpankle(1:discretization(1)+1)=pankinit1(2);

xApzmp_=[xApzmp];
yApzmp_=[yApzmp];
xApankle_=[zeros(size(xApzmp_,1),nbparamABCD+4) xApankle zeros(size(xApzmp,1),nbparamBb) zeros(size(yApzmp_))];
yApankle_=[zeros(size(xApzmp_)) zeros(size(yApzmp_,1),nbparamABCD+4) yApankle zeros(size(yApzmp,1),nbparamBb)];

xApzmp_=[xApzmp_ zeros(size(xApzmp_))];
yApzmp_=[zeros(size(yApzmp_)) yApzmp_];

Acons1=zeros(size(xApankle_));
Bcons1=zeros(size(xBpankle));
Acons2=zeros(size(xApankle_));
Bcons2=zeros(size(xBpankle));
Acons3=zeros(size(xApankle_));
Bcons3=zeros(size(xBpankle));
Acons4=zeros(size(xApankle_));
Bcons4=zeros(size(xBpankle));

j=1;
%constraint direction inverse clock-wise
theta_=theta(1:end-1);
theta_=theta_(any(type_phase==0,1));

t=theta_(j);
dx=[cos(t) -sin(t) -cos(t) sin(t)];
dy=[sin(t) cos(t) -sin(t) -cos(t)];

xA=xApzmp_-xApankle_;
yA=yApzmp_-yApankle_;
xB=xBpzmp-xBpankle;
yB=yBpzmp-yBpankle;

discretization_=discretization(any(type_phase==0,1));

for i=1:sum(discretization_)+1
    if(i>sum(discretization_(1:j))+1)
            j=j+1;
            %constraint direction inverse clock-wise
            t=theta_(j);
            dx=[cos(t) -sin(t) -cos(t) sin(t)];
            dy=[sin(t) cos(t) -sin(t) -cos(t)];
    end
    
%     if type_phase(j)==0
        Acons1(i,:)=dx(1)*(xA(i,:))+dy(1)*(yA(i,:));
        Bcons1(i)=-dx(1)*(xB(i,:))-dy(1)*(yB(i,:))+fronttoankle-sole_margin;
        Acons2(i,:)=dx(2)*(xA(i,:))+dy(2)*(yA(i,:));
        Bcons2(i)=-dx(2)*(xB(i,:))-dy(2)*(yB(i,:));
        Acons3(i,:)=dx(3)*(xA(i,:))+dy(3)*(yA(i,:));
        Bcons3(i)=-dx(3)*(xB(i,:))-dy(3)*(yB(i,:))+backtoankle-sole_margin;
        Acons4(i,:)=dx(4)*(xA(i,:))+dy(4)*(yA(i,:));
        Bcons4(i)=-dx(4)*(xB(i,:))-dy(4)*(yB(i,:));

%         Bcons2(i)=Bcons2(i)+(inttoankle*(any(mod(j,6)==[0;1;5]))+exttoankle*(any(mod(j,6)==[3;2]))-sole_margin);
%         Bcons4(i)=Bcons4(i)+(inttoankle*(any(mod(j,6)==[3;2]))  +exttoankle*(any(mod(j,6)==[0;1;5]))-sole_margin);
%         if(firstSS==0)
%             Bcons2(i)=Bcons2(i)+(inttoankle*(any(mod(j,6)==[0;1;5]))+exttoankle*(any(mod(j,6)==[3;2]))-sole_margin);
%             Bcons4(i)=Bcons4(i)+(inttoankle*(any(mod(j,6)==[3;2]))  +exttoankle*(any(mod(j,6)==[0;1;5]))-sole_margin);
%         elseif(firstSS==1)
%             Bcons2(i)=Bcons2(i)+(inttoankle*(any(mod(j,6)==[3;2]))    +exttoankle*(any(mod(j,6)==[0;1;5]))-sole_margin);
%             Bcons4(i)=Bcons4(i)+(inttoankle*(any(mod(j,6)==[0;1;5]))  +exttoankle*(any(mod(j,6)==[3;2]))-sole_margin);
%         else
%             'Choose a first SS foot'
%         end
        if(firstSS==0)
            Bcons2(i)=Bcons2(i)+(inttoankle*(mod(j,2)==1)+exttoankle*(mod(j,2)==0)-sole_margin);
            Bcons4(i)=Bcons4(i)+(inttoankle*(mod(j,2)==0)+exttoankle*(mod(j,2)==1)-sole_margin);
        elseif(firstSS==1)
            Bcons2(i)=Bcons2(i)+(inttoankle*(mod(j,2)==0)+exttoankle*(mod(j,2)==1)-sole_margin);
            Bcons4(i)=Bcons4(i)+(inttoankle*(mod(j,2)==1)+exttoankle*(mod(j,2)==0)-sole_margin);
        else
            'Choose a first SS foot'
        end

%     end
end

%  Acons=[Acons1(any(any(xApankle,2)+any(xBpankle,2),2),:);
%      Acons2(any(any(xApankle,2)+any(xBpankle,2),2),:);
%      Acons3(any(any(xApankle,2)+any(xBpankle,2),2),:);
%      Acons4(any(any(xApankle,2)+any(xBpankle,2),2),:)];
%  Bcons=[Bcons1(any(any(xApankle,2)+any(xBpankle,2),2),:);
%      Bcons2(any(any(xApankle,2)+any(xBpankle,2),2),:);
%      Bcons3(any(any(xApankle,2)+any(xBpankle,2),2),:);
%      Bcons4(any(any(xApankle,2)+any(xBpankle,2),2),:);];
 Acons=[Acons1;
     Acons2;
     Acons3;
     Acons4];
 Bcons=[Bcons1;
     Bcons2;
     Bcons3;
     Bcons4];

end