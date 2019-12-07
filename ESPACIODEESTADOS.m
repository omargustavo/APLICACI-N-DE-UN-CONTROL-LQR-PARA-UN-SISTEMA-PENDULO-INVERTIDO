M = 0.519;
m = 0.013;
b = 0.1;
I = 0.001;
g = 9.8;
l = 0.43;
p = I*(M+m)+M*m*l^2; 
A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0]
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p]
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1]

D = [0;0;0;0]

Ts=5/1000;
[F,G,H,J]=c2dm (A,B,C,D,Ts,'zoh')
co = ctrb (F,G);
ob = obsv (F,H);
Controllability = rank (co)
Observability = rank (ob)

T=0:0.005:10;
U=0.2*ones(size(T));

x=8000; %factor de peso para la posición del carro
y=600; %factor de peso para el ángulo del péndulo
Q=[x 0 0 0;
0 0 0 0;
0 0 y 0;
0 0 0 0];
R =1;
K = dlqr(F,G,Q,R)

[Y,X]=dlsim(F-G*K,G,H,J,U);
stairs(T,Y)
legend('Cart (x)','Pendulum (phi)')

%sist = ss(A,B,C,D,0.005)

%K = lqr(A,B,Q,R)
 %[K,P] = dlqr(A,B,Q,R)
 %sist = ss(A,B,C,D,0.005)
