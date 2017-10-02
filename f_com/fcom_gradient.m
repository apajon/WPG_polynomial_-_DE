function [A B] = fcom_gradient(Apzmp,Bpzmp,Apcom,Bpcom,mg,h)

A=(Apzmp-Apcom)*(-mg)/h;
B=(Bpzmp-Bpcom)*(-mg)/h;
end