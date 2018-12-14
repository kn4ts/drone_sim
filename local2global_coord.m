%% 機体座標系からグローバル座標系への変換関数
%%
function Vector_g = local2global_coord(X,Vector_l)

    %% 回転行列
    R.the   = [ 1  0  0 ;
                0  cos(X(4))  sin(X(4));
                0  -sin(X(4))  cos(X(4))];

    R.phi   = [ cos(X(5))  0  sin(X(5));
                0          1  0        ;
               -sin(X(5))  0  cos(X(5))];

    R.psi   = [ cos(X(6))  sin(X(6))  0;
               -sin(X(6))  cos(X(6))  0;
                0          0          1];

    Vector_g = R.psi * R.phi * R.the * Vector_l ;

end