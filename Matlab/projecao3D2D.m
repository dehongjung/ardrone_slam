% Variaveis 
syms roll pitch yaw
syms xf yf zf xc yc zc
syms Cx Cy fx fy

% Posicao Camera
Pc = [[xc];[yc];[zc]];

% Posicao Feature
Pf = [[xf];[yf];[zf]];

% Matriz de Rotacao (Camera)
Rc = [
    [cos(yaw)*cos(pitch), -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll), sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll)];
	[sin(yaw)*cos(pitch), cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll), -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll)];
	[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]
    ];

Rt = transpose(Rc);

aux = -Rt*Pc;

% Matriz de Rotacao (Feature)
Rf = [
    [cos(0)*cos(0), -sin(0)*cos(0) + cos(0)*sin(0)*sin(0), sin(0)*sin(0) + cos(0)*sin(0)*cos(0)];
	[sin(0)*cos(0), cos(0)*cos(0) + sin(0)*sin(0)*sin(0), -cos(0)*sin(0) + sin(0)*sin(0)*cos(0)];
	[-sin(0), cos(0)*sin(0), cos(0)*cos(0)]
    ];

% Matriz de Transformacao Homogenea
inv_H_cw = [
            [Rt(1,1) Rt(1,2) Rt(1,3) aux(1)];
            [Rt(2,1) Rt(2,2) Rt(2,3) aux(2)];
            [Rt(3,1) Rt(3,2) Rt(3,3) aux(3)];
            [0 0 0 1]
            ];
        
H_pw = [
        [Rf(1,1) Rf(1,2) Rf(1,3) xf];
        [Rf(2,1) Rf(2,2) Rf(2,3) yf];
        [Rf(3,1) Rf(3,2) Rf(3,3) zf];
        [0 0 0 1]
        ];

H_pc = inv_H_cw*H_pw;

% Posicao Relativa
X_rel = H_pc(1,4);
Y_rel = H_pc(2,4);
Z_rel = H_pc(3,4);

% Projecao 3D -> 2D (u,v)
u = Cx + (fx*X_rel)/Z_rel;
v = Cy + (fy*Y_rel)/Z_rel;

%Derivadas parciais
diff_u_xc = diff (u, xc)
diff_u_yc = diff (u, yc)
diff_u_zc = diff (u, zc)
diff_u_roll = diff (u, roll)
diff_u_pitch = diff (u, pitch)
diff_u_yaw = diff (u, yaw)
diff_u_xf = diff (u, xf)
diff_u_yf = diff (u, yf)
diff_u_zf = diff (u, zf)

diff_v_xc = diff (v, xc)
diff_v_yc = diff (v, yc)
diff_v_zc = diff (v, zc)
diff_v_roll = diff (v, roll)
diff_v_pitch = diff (v, pitch)
diff_v_yaw = diff (v, yaw)
diff_v_xf = diff (v, xf)
diff_v_yf = diff (v, yf)
diff_v_zf = diff (v, zf)

