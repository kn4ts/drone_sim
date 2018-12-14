%% ドローンの描画関数
function show_drone(t,X)

    clf reset   %% 描画域のリセット

    %%
    %% 表示時間の描画設定
    %% （フォント，サイズ）
    TEXT_OPTION = struct('FontName', 'Times', 'FontSize', 20);

    %% グローバル座標軸の描画設定
    %% （線の太さ）
    LINE_OPTION.Global = struct('linewidth', 2);
    
    %% ドローンボディの描画設定
    LINE_OPTION.Drone_X = struct('color','r','linewidth', 3);
    LINE_OPTION.Drone_Y = struct('linewidth', 3);
    LINE_OPTION.Drone_Shadow = struct('color',[0.5 0.5 0.5],'linewidth', 3);

    %%
    %% 表示文字（各軸目盛り，ラベル）の設定
    %% （フォント，サイズ）    
    AXIS_OPTION  = struct('FontSize',12,'FontName','Times');
    LABEL_OPTION = struct('FontSize',15,'FontName','Times');

    %%
    %% マーカ（射影）の設定
    %% （サイズ，色）
    MARKER_OPTION = struct('SizeData',10,'MarkerEdgeColor','none','MarkerFaceColor','k');

    %%
    %% 描画に関するドローンのパラメータ設定
    DRONE_LEN = 1 ; %% ドローン全長[m]
    % DRONE_THI = 0.08; %% ドローン厚み[m]  %% 未実装
    % DRONE_RAD = 0.1 ; %% プロペラ半径[m]  %% 未実装

    %%
    %% プロット領域の設定
    axis_LIMIT = [ -5 5 -5 5 0 10]; %% X,Y,Z軸の表示限界値

    %%
    %% 地面表示のための設定
    Lim_x.plus =  0.9*axis_LIMIT(2);
    Lim_x.minu =  0.9*axis_LIMIT(1);
    Lim_y.plus =  0.9*axis_LIMIT(4);
    Lim_y.minu =  0.9*axis_LIMIT(3);
    Lim_z.plus =  axis_LIMIT(6);
    Lim_z.minu =  axis_LIMIT(5);
    [Grid.X,Grid.Y] = meshgrid(Lim_x.minu:.5:Lim_x.plus);
    Grid.Z = zeros(length(Grid.X),length(Grid.Y)) ; %% （z=0の平面）
    

    %%=====================================================================
    %% 姿勢・位置座標計算部
    %%

    %% ドローンの座標計算
    %% （状態X -> グローバル座標の位置，姿勢）
    %%
    %% 端点x+のグローバル座標ベクトル
    Body_X.plus = local2global_coord(X,[ DRONE_LEN ; 0 ; 0 ]) +X(1:3) ;
    %% 端点x-のグローバル座標ベクトル
    Body_X.minu = local2global_coord(X,[-DRONE_LEN ; 0 ; 0 ]) +X(1:3) ;
    %% 端点y+のグローバル座標ベクトル
    Body_Y.plus = local2global_coord(X,[ 0 ; DRONE_LEN ; 0 ]) +X(1:3) ;
    %% 端点y-のグローバル座標ベクトル
    Body_Y.minu = local2global_coord(X,[ 0 ;-DRONE_LEN ; 0 ]) +X(1:3) ;


    %%=====================================================================
    %% 描画部
    %%
    hold on

    %% フレーム描画で使うplot3関数のための変数置き換え
    BodyX.x = [ Body_X.plus(1) ; Body_X.minu(1) ] ; %% ボディXのx座標ベクトル
    BodyX.y = [ Body_X.plus(2) ; Body_X.minu(2) ] ; %% ボディXのy座標ベクトル
    BodyX.z = [ Body_X.plus(3) ; Body_X.minu(3) ] ; %% ボディXのz座標ベクトル
    BodyY.x = [ Body_Y.plus(1) ; Body_Y.minu(1) ] ; %% ボディYのx座標ベクトル
    BodyY.y = [ Body_Y.plus(2) ; Body_Y.minu(2) ] ; %% ボディYのy座標ベクトル
    BodyY.z = [ Body_Y.plus(3) ; Body_Y.minu(3) ] ; %% ボディYのz座標ベクトル

    %% Droneのフレーム描画
    plot3(BodyX.x, BodyX.y, BodyX.z, LINE_OPTION.Drone_X);
    plot3(BodyY.x, BodyY.y, BodyY.z, LINE_OPTION.Drone_Y);

    %% 平面への射影プロット
    %% 重心座標の射影描画
    % scatter3(X(1),X(2),axis_LIMIT(5), MARKER_OPTION); %% X-Y平面への射影
    scatter3(X(1),X(2),axis_LIMIT(5), 'SizeData',70,'MarkerEdgeColor','none','MarkerFaceColor',[0.5 0.5 0.5]); %% X-Y平面への射影
    scatter3(X(1),axis_LIMIT(4),X(3),'SizeData',70,'MarkerEdgeColor','none','MarkerFaceColor',[0.5 0.5 0.5]); %% X-Z平面への射影
    scatter3(axis_LIMIT(2),X(2),X(3),'SizeData',70,'MarkerEdgeColor','none','MarkerFaceColor',[0.5 0.5 0.5]); %% Y-Z平面への射影

    %% フレームの射影描画
    plot3(BodyX.x, BodyX.y, [axis_LIMIT(5);axis_LIMIT(5)], LINE_OPTION.Drone_Shadow); %% X-Y平面への射影(Body X)
    plot3(BodyY.x, BodyY.y, [axis_LIMIT(5);axis_LIMIT(5)], LINE_OPTION.Drone_Shadow); %% X-Y平面への射影(Body Y)
    plot3(BodyX.x, [axis_LIMIT(4);axis_LIMIT(4)], BodyX.z,LINE_OPTION.Drone_Shadow); %% X-Z平面への射影
    plot3([axis_LIMIT(2);axis_LIMIT(2)], BodyY.y, BodyY.z, LINE_OPTION.Drone_Shadow); %% Y-Z平面への射影

    %% 地面の描画
    %% GROUND
    mesh(Grid.X,Grid.Y,Grid.Z); %% 地面をメッシュで表示
    % rectangle('Position', [ -5  -5  15  5 ], 'FaceColor', [ 0.5  0.5  0.5], 'EdgeColor', 'none');

    %% グローバル座標軸の描画
    % axis_LIMIT(1:2)
    line(axis_LIMIT(1:2),zeros(2),LINE_OPTION.Global);
    line(zeros(2),axis_LIMIT(3:4),LINE_OPTION.Global);

    %% 経過時間の描画
    %% TIME INDICATOR
    TimeIndicate = strcat(num2str(t,'%2.2f'),' [sec]');
    % text(0.5+LEFT_END,4,TimeIndicate,TEXT_OPTION);
    text(Lim_x.minu,Lim_y.plus,0.1*Lim_z.plus,TimeIndicate,TEXT_OPTION);
    % text(Lim_x.minu,Lim_y.plus,2,TimeIndicate,TEXT_OPTION);

    %% プロット領域の調整
    %% PLOT AREA
    grid on              %% グリッドを表示
    axis equal           %% X,Y,Z軸を等スケールで表示
    axis(axis_LIMIT)     %% X,Y,Z軸の表示限界値
    h = gca;
    set(h,'xtick',axis_LIMIT(1):1:axis_LIMIT(2),AXIS_OPTION)
    set(h,'ytick',axis_LIMIT(3):1:axis_LIMIT(4),AXIS_OPTION)
    set(h,'ztick',axis_LIMIT(5):1:axis_LIMIT(6),AXIS_OPTION)
    set(get(gca, 'XLabel'), 'String', 'x [m]',LABEL_OPTION);
    set(get(gca, 'YLabel'), 'String', 'y [m]',LABEL_OPTION);
    set(get(gca, 'ZLabel'), 'String', 'z [m]',LABEL_OPTION);

    %% 視点の調整
    %% VIEW POINT
    view(3)

    hold off
    drawnow %% Figureの更新
    % getframe(gca)
end