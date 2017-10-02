function [xlA xlB ylA ylB xuA xuB yuA yuB] = zmp_contraint_SSP_xy2(pankinit,pankfin,discretization,backtoankle,fronttoankle,exttoankle,inttoankle,sole_margin)

% nbviapoints=length(tpassage);
nbphases=length(discretization);

xlA=[];
ylA=[];
xuA=[];
yuA=[];
xlB=[];
ylB=[];
xuB=[];
yuB=[];

for i=1:discretization(2)
    xlB=[xlB;-(backtoankle-sole_margin)+pankinit(1)];
    ylB=[ylB;-(inttoankle-sole_margin)+pankinit(2)];
    %             l=[l;-(backtoankle-sole_margin)+pstep(ceil(j/3),1) -(inttoankle*(mod(j,6)==1)+exttoankle*(mod(j,6)==4)-sole_margin)+pstep(ceil(j/3),2)];
    
    xuB=[xuB;(fronttoankle-sole_margin)+pankinit(1)];
    yuB=[yuB;(exttoankle-sole_margin)+pankinit(2)];
    %             u=[u;(fronttoankle-sole_margin)+pstep(ceil(j/3),1) (inttoankle*(mod(j,6)==4)+exttoankle*(mod(j,6)==1)-sole_margin)+pstep(ceil(j/3),2)];
end

xlA=[xlA;zeros(discretization(2),1)];
ylA=[ylA;zeros(discretization(2),1)];
xuA=[xuA;zeros(discretization(2),1)];
yuA=[yuA;zeros(discretization(2),1)];

for j=4:nbphases-2
    nb=discretization(j);
    if(mod(j,3)==0)
%         for i=1:nb
%         %l=[l;0 0];
%         %u=[u;0 0];
%         end
    elseif(mod(j,3)==1)
        for i=1:nb
            xlB=[xlB;-(backtoankle-sole_margin)];
            ylB=[ylB;-(inttoankle*(mod(j,6)==1)+exttoankle*(mod(j,6)==4)-sole_margin)];
            
            xlA(end+1,ceil(j/3)-1)=1;
            ylA(end+1,ceil(j/3)-1)=1;
%             l=[l;-(backtoankle-sole_margin)+pstep(ceil(j/3),1) -(inttoankle*(mod(j,6)==1)+exttoankle*(mod(j,6)==4)-sole_margin)+pstep(ceil(j/3),2)];
            xuB=[xuB;(fronttoankle-sole_margin)];
            yuB=[yuB;(inttoankle*(mod(j,6)==4)+exttoankle*(mod(j,6)==1)-sole_margin)];
            
            xuA(end+1,ceil(j/3)-1)=1;
            yuA(end+1,ceil(j/3)-1)=1;
%             u=[u;(fronttoankle-sole_margin)+pstep(ceil(j/3),1) (inttoankle*(mod(j,6)==4)+exttoankle*(mod(j,6)==1)-sole_margin)+pstep(ceil(j/3),2)];
        end
    elseif(mod(j,3)==2)
        for i=1:nb
            xlB=[xlB;-(backtoankle-sole_margin)];
            ylB=[ylB;-(inttoankle*(mod(j,6)==2)+exttoankle*(mod(j,6)==5)-sole_margin)];
            
            xlA(end+1,ceil(j/3)-1)=1;
            ylA(end+1,ceil(j/3)-1)=1;
%             l=[l;-(backtoankle-sole_margin)+pstep(ceil(j/3),1) -(inttoankle*(mod(j,6)==2)+exttoankle*(mod(j,6)==5)-sole_margin)+pstep(ceil(j/3),2)];
            xuB=[xuB;(fronttoankle-sole_margin)];
            yuB=[yuB;(inttoankle*(mod(j,6)==5)+exttoankle*(mod(j,6)==2)-sole_margin)];
            
            xuA(end+1,ceil(j/3)-1)=1;
            yuA(end+1,ceil(j/3)-1)=1;
%             u=[u;(fronttoankle-sole_margin)+pstep(ceil(j/3),1) (inttoankle*(mod(j,6)==5)+exttoankle*(mod(j,6)==2)-sole_margin)+pstep(ceil(j/3),2)];
        end
    end
end



for i=1:discretization(end-1)
    xlB=[xlB;-(backtoankle-sole_margin)+pankfin(1)];
    ylB=[ylB;-(inttoankle*(mod(nbphases-1,6)==1)+exttoankle*(mod(nbphases-1,6)==4)-sole_margin)+pankfin(2)];
    %             l=[l;-(backtoankle-sole_margin)+pstep(ceil(j/3),1) -(inttoankle*(mod(j,6)==2)+exttoankle*(mod(j,6)==5)-sole_margin)+pstep(ceil(j/3),2)];
    
    xuB=[xuB;(fronttoankle-sole_margin)+pankfin(1)];
    yuB=[yuB;(inttoankle*(mod(nbphases-1,6)==4)+exttoankle*(mod(nbphases-1,6)==1)-sole_margin)+pankfin(2)];
    %             u=[u;(fronttoankle-sole_margin)+pstep(ceil(j/3),1) (inttoankle*(mod(j,6)==5)+exttoankle*(mod(j,6)==2)-sole_margin)+pstep(ceil(j/3),2)];
end

xlA=[xlA;zeros(discretization(end-1),size(xlA,2))];
ylA=[ylA;zeros(discretization(end-1),size(ylA,2))];
xuA=[xuA;zeros(discretization(end-1),size(xuA,2))];
yuA=[yuA;zeros(discretization(end-1),size(yuA,2))];




end