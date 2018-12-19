function X_d = drone_sim(X,T)

    M = 1; %% 質量[kg]
    L = 1; %% アームの長さ[m]
    J.b1 = 0.1467 ;  %% theta, phi に関する慣性モーメント
    J.b2 = 0.02331 ; %% psi に関する慣性モーメント
    J.m  = 175.69 * 10^-6 ; %% ロータの慣性モーメント
    g = 9.81 ; %% 重力加速度
    coef.prop = 4.34*10^-5 ;    %% 推力係数
    coef.drag = 0.2188*10^-5 ;  %% 抗力係数

    omega_exp2 = T/coef.prop ;  %% プロペラ角速度の2乗をまとめたベクトル
    omega = omega_exp2.^(0.5);  %% プロペラ角速度をまとめたベクトル
    Omega = omega(1)-omega(2)+omega(3)-omega(4) ;
    tau = (coef.prop/coef.drag)*T ; %% プロペラ反作用トルクをまとめたベクトル

    A_xu = [ X(7:12) ;
             (cos(X(4))*sin(X(5))*cos(X(6))+sin(X(4))*sin(X(5)))*sum(T)/M;
             (cos(X(4))*sin(X(5))*sin(X(6))+sin(X(4))*cos(X(5)))*sum(T)/M;
             cos(X(4))*cos(X(5))*sum(T)/M;
            -(J.b1-J.b2)*X(10)*X(12)/J.b1-J.m*X(10)*Omega/J.b1 ;
             (J.b1-J.b2)*X(11)*X(12)/J.b1-J.m*X(11)*Omega/J.b1 ;
             0 ];

    B_u = [  zeros(8,1) ;
            -g ;
             L*(0+T(2)+0-T(4))/J.b1 ;
             L*(-T(1)+0+T(3)+0)/J.b1 ;
             (+tau(1)-tau(2)+tau(3)-tau(4))/J.b2 ];

    X_d = A_xu +B_u ;    

end