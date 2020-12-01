
%%apply the 3D pathloss function to CDMA model 
function [pl_r] = pathloss3D(R, h)
a = 9.61;
b = 0.16;
eta_los = 1;
eta_Nlos = 20;
f_c = 2.4*10^9;
 c = 3*10^8;
A = eta_los-eta_Nlos;
B = 20*log10(f_c) + 20*log10(4*pi/c) + eta_Nlos;
pl = A/(1+a*exp(-b*(atand(h/R)-a)))+10*log10(h^2+R^2)+ B;
pl_r = 10^(pl/10);

end

