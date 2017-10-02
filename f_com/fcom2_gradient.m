function [A B] = fcom2_gradient(Afcom,Bfcom)

% [Afcom Bfcom]=fcom_gradient(Apzmp,Bpzmp,Apcom,Bpcom,mg,h);

A=transpose(Afcom)*Afcom;
B=transpose(Bfcom)*Afcom;

end