M = 0.518;
m = 0.017;
b = 0.1;
I = 0.002;
g = 9.8;
l = 0.43;
p = I*(M+m)+M*m*l^2; 

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

estados = {'x' 'x_punto' 'phi' 'phi_punto'};
entrada = {'u'};
salidas = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',estados,'inputname',entrada,'outputname',salidas)
co = ctrb(sys_ss)
controlabilidad = rank(co)
Q = C'*C
R = 1;
Q(1,1) = 3000
Q(3,3) = 100
[Ad,Bd,Cd,Dd] = c2dm(A,B,C,D,0.005,'zoh')

K = dlqr(Ad,Bd,Q,R)

An = [(Ad-Bd*K)];
Bn = [Bd];
Cn = [Cd];
Dn = [Dd];

estados = {'x' 'x_dot' 'phi' 'phi_dot'};
entrada = {'r'};
salidas = {'x'; 'phi'};

%%sys_cl = ss(An,Bn,Cn,Dn,'statename',estados,'inputname',entrada,'outputname',salidas)

t = 0:0.01:5;
r =0.2*ones(size(t));
[Y,X]=dlsim(An,Bn,Cn,Dn,r);
stairs(t,Y)
legend('Carro(x)','Pendulo(phi)')
%%[y,t,x]=lsim(sys_cl,r,t);
%%[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
%%set(get(AX(1),'Ylabel'),'String','posicion del carrito (m)')
%%set(get(AX(2),'Ylabel'),'String','angulo del pendulo (radianes)')
%%title('Respuesta al escalon con control LQR'