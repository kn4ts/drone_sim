%% 変数ワークスペース，Figureの初期化
clear all
close all

%%
%% 位置，機体角度の初期値設定
%%
%% X = [ x; y; z; theta; phi; psi; x_dot; y_dot; z_dot; theta_dot; phi_dot; psi_dot ];
%% x : 重心位置のx座標
%% y : 重心位置のy座標
%% z : 重心位置のz座標
%% theta : x_G軸（グローバル座標系）に対する機体座標系の角度
%% phi   : y_G軸（グローバル座標系）に対する機体座標系の角度
%% psi   : z_G軸（グローバル座標系）に対する機体座標系の角度
%%
X_0 = [ 0; -1; 0; 0.05 ; 0.1 ; 1 ; zeros(6,1) ];

%%
%% シミュレーション条件の設定
%% SIMULATION CONDITIONS
%% offset = 1;
T_end = 10;     %% シミュレーション終了時刻 [sec]
T_sam = 0.01;   %% サンプリング時間 [sec]
% t = 0.0;        %% 
tt = 0:T_sam:T_end-T_sam ;
N_all = length(tt) ;

r = 6;  %% 目標高度[m]
        %% 目標位置はx=0,y=0としている

%%
%% PID Gains
%%
%% for attitude control
Kp.the = 1 ;
Kp.phi = Kp.the ;
Kp.psi = 2 ;

Ki.the = 0.5 ;
Ki.phi = Ki.the ;
Ki.psi = 1 ;

Kd.the = 0.1 ;
Kd.phi = Kd.the ;
Kd.psi = 3 ;

%%
%% for position control
Kp.x = 0.2 ;
Kp.y = 0.2 ;
Kp.z = 1 ;
% Kp.z = 1 ;

Ki.x = 0.1 ;
Ki.y = 0.1 ;
Ki.z = 0.5 ;

Kd.x = 0.05 ;
Kd.y = 0.05 ;
Kd.z = 0.7 ;

%% 信号の初期化
%% SIGNALS
X = X_0 ;
Y = X ;
err.angle = zeros(3,1)-X(4:6);
err_int.angle = zeros(3,1);
err.posit = [ 0 ; 0 ; r ]-X(1:3);
err_int.posit = 0;
% x = [ x_0 ; xd_0 ];
% y = C*x ;
i = 1;

%% 動画保存の設定 !開発中! 一応使えるが図が点滅する...
% SAVE_MOVIE = 0; %% 動画を保存するかどうか
%                 %% 0 : 保存しない
%                 %% 1 : 保存する
% NAME_OF_MOVIE = 'kika3D.mp4';   %% 動画保存ファイル名

% 初期値を表示して一時停止
% figure(1)
show_drone(0,X);
disp('Press any key to start simulation')
pause()


% メインループ
% MAIN LOOP
for j = 1:N_all
    
    %%/////////////////////////////////////////////////////////////////////////
    %% PID制御則
    %%
    %% 概要．姿勢角と位置をフィードバックするPID制御則を構成
    %%
    %% 1. x,y,z（位置）および theta,phi,psi（姿勢）に関する6つの制御信号を個別に生成
    %% 2. それらの制御信号を混合(mixing)し，ドローンの4つのロータ推力に分配
    %% 
    
    %% 姿勢制御PID
    %% （theta,phi,psi）
    err.angle_now = zeros(3,1)-X(4:6) ;                %% 現ステップの姿勢誤差計算
    err_der.angle = (err.angle_now -err.angle)/T_sam ; %% 姿勢誤差の数値微分計算
    err_int.angle = err_int.angle +T_sam*err.angle ;   %% 姿勢誤差の数値積分計算
    err.angle = err.angle_now; %% 現ステップ誤差を更新
    %% 姿勢角度に関するPID入力を計算
    U_angle =  diag([Kp.the Kp.phi Kp.psi])*err.angle ...       
              +diag([Ki.the Ki.phi Ki.psi])*err_int.angle ...       
              +diag([Kd.the Kd.phi Kd.psi])*err_der.angle ;     

    %% 位置制御PID
    %% （x,y,z）
    err.posit_now = [ 0 ; 0 ; r ] -X(1:3) ;            %% 現ステップの位置誤差計算
    err_der.posit = (err.posit_now -err.posit)/T_sam ; %% 位置誤差の数値微分計算
    err_int.posit = err_int.posit +T_sam*err.posit ;   %% 位置誤差の数値積分計算
    err.posit = err.posit_now; %% 現ステップ誤差を更新
    %% 位置に関するPID入力を計算
    U_posit =  diag([Kp.x Kp.y Kp.z])*err.posit ...
              +diag([Ki.x Ki.y Ki.z])*err_int.posit ...
              +diag([Kd.x Kd.y Kd.z])*err_der.posit ;

    %% 制御分配
    %% mixing
    %% [ ロータ推力ベクトル ] = [ 混合・分配行列 ] * [ 姿勢角PID信号 ; 位置PID信号 ]
    %%      [ U_rotor ]     = [    定数行列    ] * [ U_angle ; U_posit ]
    %% ここで[混合・分配行列]は，ドローンの幾何構造，各ロータの回転方向より決定した
    U_rotor = [ 0  -1   1  -1   0   1 ;
                1   0  -1   0   1   1 ;
                0   1   1   1   0   1 ;
               -1   0  -1   0  -1   1 ] * [ U_angle ; U_posit ];
    %%
    %%/////////////////////////////////////////////////////////////////////////
    
    %% プラントのシミュレーション
    X_d = drone_sim(X,U_rotor);
    X   = X + T_sam * X_d ;
    % y = C * x ;

    %% 信号の保存
    X_h(:,j) = X;
    T_h(:,j) = U_rotor;
    % st = sin(3*T_sam*j);
    % ct = cos(3*T_sam*j);

    %%
    %% 描画部分
    if (rem(j,10)==0)           %% 描画頻度の設定
        show_drone(j*T_sam,X);  %% Droneを描画
        % if SAVE_MOVIE == 1    %% 動画保存（開発中）
        %     F(i) = getframe(gcf);
        %     i = i+1;
        % end
    end

    if X(3)<0   %% 墜落判定
        disp('Drone has fallen!')
        break;
    end
end

%% 結果の表示
figure(2)

subplot(2,2,1)
plot(X_h(1:3,:)')
legend('x','y','z','Location','best')
title('Position X, Y, Z [m]')
xlabeltext = strcat('step ( 1 step = ', num2str(T_sam,'%2.2f'),' [sec] )');
xlabel(xlabeltext)
subplot(2,2,3)
plot(X_h(4:6,:)')
legend('\theta','\phi','\psi','Location','best')
title('Attitude angle \theta, \phi, \psi [rad]')
xlabel(xlabeltext)

subplot(4,2,2)
plot(T_h(1,:)')
title('Thrust (propeling force) T_1, T_2, T_3, T_4 [N]')
ylabel('T_1 [N]')
subplot(4,2,4)
plot(T_h(2,:)')
ylabel('T_2 [N]')
subplot(4,2,6)
plot(T_h(3,:)')
ylabel('T_3 [N]')
subplot(4,2,8)
plot(T_h(4,:)')
ylabel('T_4 [N]')
xlabel(xlabeltext)


%% 動画の保存（開発中）
%% ！注意：MATLAB2016以降で有効！ <- 2013bでも動作確認
% if SAVE_MOVIE == 1
%     disp('Saving movie file...')
%     v = VideoWriter(NAME_OF_MOVIE,'MPEG-4');
%     open(v);
%     writeVideo(v,F);
%     close(v);
%     disp('done.')
% end