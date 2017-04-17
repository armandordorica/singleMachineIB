%%%2.8 The power supply was turned on and the following values were measured: 
E1 = 215  %in volts
E2 = 245   % in volts
I1 = 3.4 % in Amps 
startingTorque = 22 %lbf-in 

S = sqrt(3)*E1*I1

I1 = [1.1
0.9
0.7
0.5
0.32
0.2
0.12
0.22
0.38
]

I2 = [
    0
0.1
0.2
0.3
0.4
0.5
0.6
0.7
0.8]

plot(I2, I1, '-o')
title('AC stator current values against DC rotor excitation current values')
ylabel('AC stator current values')
xlabel('DC rotor excitation current values')


pf = [ 
0.1708859148
0.1491861161
0.1342675045
0.1611210054
0.1678343806
0.2014012567
0.4028025134
0.2807411457
0.2120013228]


plot(I2, pf, '-o')
title('Recorded power factors against the DC rotor excitation current values')
ylabel('Power Factor')
xlabel('DC rotor excitation current values')