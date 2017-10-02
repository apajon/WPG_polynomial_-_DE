%Compute the matrix gradient of A vector
function [A]=com_morisawa_A_gradient(nbphases,w)
A=[];
for j=1:nbphases
    Aj=[1 0 2/w^2 0 24/w^4 0;
        0 1 0 6/w^2 0 120/w^4;
        0 0 1 0 12/w^2 0;
        0 0 0 1 0 20/w^2;
        0 0 0 0 1 0;
        0 0 0 0 0 1];
    A=[A;zeros(6,(j-1)*6) Aj zeros(6,(nbphases-j)*6)];
end
end