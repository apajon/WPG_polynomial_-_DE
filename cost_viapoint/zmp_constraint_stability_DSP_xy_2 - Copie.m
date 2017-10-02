function [Acons Bcons] = zmp_constraint_stability_DSP_xy_2(xApzmp,xBpzmp,yApzmp,yBpzmp,pankinit,pankfin1,pankfin2,discretization,backtoankle,fronttoankle,exttoankle,inttoankle,sole_margin,theta,type_phase,nbparamBb)

nbstep=length(discretization);
[xApankle xBpankle]=torque_ankle_positions_DSP_22(pankinit(1),pankfin1(1),discretization);
[yApankle yBpankle]=torque_ankle_positions_DSP_22(pankinit(2),pankfin1(2),discretization);
% yBpankle(end-discretization(end)+1:end)=yBpankle(end-discretization(end)+1:end)-(-1)^(nbstep)*0.095*2;
xBpankle(end-discretization(end)+1:end)=pankfin2(1);
yBpankle(end-discretization(end)+1:end)=pankfin2(2);


xApzmp_=[xApzmp];
yApzmp_=[yApzmp];
xApankle=[zeros(size(xApzmp_,1),size(xApzmp_,2)-size(xApankle,2)-nbparamBb) xApankle zeros(size(xApzmp,1),nbparamBb) zeros(size(yApzmp_))];
yApankle=[zeros(size(xApzmp_)) zeros(size(yApzmp_,1),size(yApzmp_,2)-size(yApankle,2)-nbparamBb) yApankle zeros(size(yApzmp,1),nbparamBb)];

xApzmp_=[xApzmp_ zeros(size(xApzmp_))];
yApzmp_=[zeros(size(yApzmp_)) yApzmp_];

Acons1=zeros(size(xApankle));
Bcons1=zeros(size(xBpankle));
Acons2=zeros(size(xApankle));
Bcons2=zeros(size(xBpankle));
Acons3=zeros(size(xApankle));
Bcons3=zeros(size(xBpankle));
Acons4=zeros(size(xApankle));
Bcons4=zeros(size(xBpankle));

j=1;
%constraint direction inverse clock-wise
t=theta(j+1);
dx=[cos(t) -sin(t) -cos(t) sin(t)];
dy=[sin(t) cos(t) -sin(t) -cos(t)];
for i=1:sum(discretization)+1
    if(i>sum(discretization(1:j))+1)
            j=j+1;
            %constraint direction inverse clock-wise
            t=theta(j+1);
            dx=[cos(t) -sin(t) -cos(t) sin(t)];
            dy=[sin(t) cos(t) -sin(t) -cos(t)];
    end
    
    if type_phase(j)==0
        Acons1(i,:)=dx(1)*(xApzmp_(i,:)-xApankle(i,:))+dy(1)*(yApzmp_(i,:)-yApankle(i,:));
        Bcons1(i)=-dx(1)*(xBpzmp(i,:)-xBpankle(i,:))-dy(1)*(yBpzmp(i,:)-yBpankle(i,:))+fronttoankle-sole_margin;
        Acons2(i,:)=dx(2)*(xApzmp_(i,:)-xApankle(i,:))+dy(2)*(yApzmp_(i,:)-yApankle(i,:));
        Bcons2(i)=-dx(2)*(xBpzmp(i,:)-xBpankle(i,:))-dy(2)*(yBpzmp(i,:)-yBpankle(i,:));
        Acons3(i,:)=dx(3)*(xApzmp_(i,:)-xApankle(i,:))+dy(3)*(yApzmp_(i,:)-yApankle(i,:));
        Bcons3(i)=-dx(3)*(xBpzmp(i,:)-xBpankle(i,:))-dy(3)*(yBpzmp(i,:)-yBpankle(i,:))+backtoankle-sole_margin;
        Acons4(i,:)=dx(4)*(xApzmp_(i,:)-xApankle(i,:))+dy(4)*(yApzmp_(i,:)-yApankle(i,:));
        Bcons4(i)=-dx(4)*(xBpzmp(i,:)-xBpankle(i,:))-dy(4)*(yBpzmp(i,:)-yBpankle(i,:));

%         Bcons2(i)=Bcons2(i)+(inttoankle*(mod(j,6)==(3||2))   +exttoankle*(mod(j,6)==(0||1||5))-sole_margin);
%         Bcons4(i)=Bcons4(i)+(inttoankle*(mod(j,6)==(0||1||5))+exttoankle*(mod(j,6)==(3||2))-sole_margin);
        Bcons2(i)=Bcons2(i)+(inttoankle*(any(mod(j,6)==[3;2]))   +exttoankle*(any(mod(j,6)==[0;1;5]))-sole_margin);
        Bcons4(i)=Bcons4(i)+(inttoankle*(any(mod(j,6)==[0;1;5]))+exttoankle*(any(mod(j,6)==[3;2]))-sole_margin);
%       Bcons2(i)=Bcons2(i)+(inttoankle*(any(mod(j,6)==[0;1;5]))+exttoankle*(any(mod(j,6)==[3;2]))-sole_margin);
%       Bcons4(i)=Bcons4(i)+(inttoankle*(any(mod(j,6)==[3;2]))  +exttoankle*(any(mod(j,6)==[0;1;5]))-sole_margin);
    end
end

 Acons=[Acons1(any(any(xApankle,2)+any(xBpankle,2),2),:);
     Acons2(any(any(xApankle,2)+any(xBpankle,2),2),:);
     Acons3(any(any(xApankle,2)+any(xBpankle,2),2),:);
     Acons4(any(any(xApankle,2)+any(xBpankle,2),2),:)];
 Bcons=[Bcons1(any(any(xApankle,2)+any(xBpankle,2),2),:);
     Bcons2(any(any(xApankle,2)+any(xBpankle,2),2),:);
     Bcons3(any(any(xApankle,2)+any(xBpankle,2),2),:);
     Bcons4(any(any(xApankle,2)+any(xBpankle,2),2),:);];

end