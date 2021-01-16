# Numerical-Inverse-Kinematics-2-link
syms t1 t2 la1 la2
%% initialize variables
del_t1 = 0; del_t2 = 0; f_pre = [0 0 0 0 0 0]';  ex = 1;    iter = 0;   xd = [0.366 1.366 0 0 0 0]';

%% DH matrix
T0 = DH0(t1);      
T = DH0(t1)*DH1(t2);

%% calculate Jacobian
p = T(1:3, 4);
Jv = [diff(p, t1) diff(p, t2)];
Jw = [T0(1:3, 3) T(1:3, 3)]

%% iteration
while ex > 0.001
iter = iter+1;
t1 = 0+del_t1; t2 = (pi/6)+del_t2;
J = [-sin(t1+t2)-sin(t1) -sin(t1+t2);
    cos(t1+t2)+cos(t1) cos(t1+t2);
    0 0;
    0 0;
    0 0;
    1 1];
psudo = pinv(J);    %psudo inverse

temp1 = DH0(t1)*DH1(t2);
f = [temp1(1, 4) temp1(2, 4) temp1(3, 4) 0 0 0]';

%% del_theta calculate
del_theta = psudo*(xd-f)

%% variables update
del_t1 = del_t1+del_theta(1, 1);   del_t2 = del_t2+del_theta(2, 1);
ex = norm(abs(f_pre-f))/norm(abs(f_pre))
f_pre = f;
end

fprintf("iteration count : %d, t1 = %d, t2 = %d", iter, round(rad2deg(t1), 4), round(rad2deg(t2), 4))
