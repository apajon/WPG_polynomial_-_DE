function [l u] = zmp_contraint_SSP_xy(pstep,discretization,backtoankle,fronttoankle,exttoankle,inttoankle,sole_margin)

% nbviapoints=length(tpassage);
nbphases=length(discretization);
%l=[0 0];
%u=[0 0];
l=[];
u=[];
for j=1:nbphases
    nb=discretization(j);
    if(mod(j,3)==0||j==1||j==nbphases)
        for i=1:nb
        %l=[l;0 0];
        %u=[u;0 0];
        end
    elseif(mod(j,3)==1)
        for i=1:nb
            l=[l;-(backtoankle-sole_margin)+pstep(ceil(j/3),1) -(inttoankle*(mod(j,6)==1)+exttoankle*(mod(j,6)==4)-sole_margin)+pstep(ceil(j/3),2)];
            u=[u;(fronttoankle-sole_margin)+pstep(ceil(j/3),1) (inttoankle*(mod(j,6)==4)+exttoankle*(mod(j,6)==1)-sole_margin)+pstep(ceil(j/3),2)];
        end
    elseif(mod(j,3)==2)
        for i=1:nb
            l=[l;-(backtoankle-sole_margin)+pstep(ceil(j/3),1) -(inttoankle*(mod(j,6)==2)+exttoankle*(mod(j,6)==5)-sole_margin)+pstep(ceil(j/3),2)];
            u=[u;(fronttoankle-sole_margin)+pstep(ceil(j/3),1) (inttoankle*(mod(j,6)==5)+exttoankle*(mod(j,6)==2)-sole_margin)+pstep(ceil(j/3),2)];
        end
    end
end


end