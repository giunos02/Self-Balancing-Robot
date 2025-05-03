
num = [9.1 0]; 
den = [1 0.16 -76.95 -8.93];

pend = tf(num,den)  %calcola la funzione di trasferimento
t=0:0.01:5;
%impulse(pend, t);
%stepplot(pend, t);


Kd = 0;
Kp = 22;
Ki = 0;
contr=tf([Kd Kp Ki],[1 0]); % FDT del controllore
sys_cl=feedback(pend,contr); % controllore in retroazione
t=0:0.01:5;
stepplot(sys_cl,t);
axis([0 3 -10 10]);
