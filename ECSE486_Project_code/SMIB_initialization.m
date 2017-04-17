%This is a script used to calculate the intial conditions for SMIB system

%Input:              IB apparent power (S_IB) and voltage magnitude (V_IB);
%                    reactances: SG (Xp), transformer (Xt), line (Xl);
%                    Number of parallel lines (nl)

%Output:             Transient EMF E' (Ep), which remains constant;
%                    internal rotor angle (delta_0);
%                    mechanical power (Pm)

%All the values are in p.u.

clear all
clear vars 
%Input your data here
S_IB = 0.93 +0.27i ;  
V_IB = 1.03 ;
% V_IB = 0.001 ;
Xp = 0.2 ;  
Xt = 0.11; 
Xl = 0.84 ; 
nl = 3;

%Calculate IB (equivalent circuit) current:
I_IB = conj(S_IB/V_IB);

%Equivalent system (circuit) impedance:
Xeq = Xp + Xt + Xl/nl;

%Find E' phasor:
Ep_phasor = V_IB+1i*Xeq*I_IB;

%Angle
delta_0 = angle(Ep_phasor);

%Magnitude
Ep = abs(Ep_phasor);

%Mechanical power
Pm = real(S_IB);


