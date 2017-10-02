function [A B] = cost_viapoint_gradient_withDSP_azmp1(Aviapoint,Bviapoint,Atorque2_1,Btorque2_1,Atorque2_2,Btorque2_2,Aa2zmp1,Ba2zmp1,lambda,mu)

A=[Aviapoint zeros(size(Aviapoint,1),size(Atorque2_1,2)-size(Aviapoint,2));zeros(size(Atorque2_1,1)-size(Aviapoint,1),size(Atorque2_1,2))]+mu*Atorque2_1+mu*Atorque2_2+lambda*Aa2zmp1;
B=[Bviapoint zeros(1,size(Btorque2_1,2)-size(Bviapoint,2))]+mu*Btorque2_1+mu*Btorque2_2+lambda*Ba2zmp1;
end