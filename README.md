# kn_draw_drone

このプロジェクトでは，ドローンの動作を可視化するコードを目指します．  
使用言語は`MATLAB2013b`です．

__＜使う方法＞__  
1.  MATLABを立ち上げ，カレントディレクトリを`kn_draw_drone`に移動する  
2.  `main.m`を実行する

![イメージ図](https://github.com/kn4ts/drone_sim/blob/ac_manuscript/draw_drone.gif)

__＜内容物＞__  
+  `README.md` ...説明書
+  `main.m` ...これを実行すれば一通り動く
+  `show_drone.m` ...ドローン描画関数  
    ```matlab
        show_drone(t,X)
        %% 出力：
        %%      なし（3次元空間内にドローンの変位，姿勢を描画）
        %% 入力：
        %%      t : 現在時刻 （秒）
        %%      X : ドローンの状態ベクトル
        %%         （変位，姿勢角，およびそれらの微分）
    ```
    +  `local2global_coord.m` ...機体座標系の位置ベクトルをグローバル座標系にする関数
        ```matlab
        V_global = local2global_coord(X,V_local)
        %% 出力：
        %%      V_global ： グローバル座標系で表現された位置ベクトル
        %% 入力：
        %%      X ： ドローンの状態ベクトル
        %%      V_local ： 機体座標系で表現された位置ベクトル
        ```

+  `drone_sim.m` ...ドローンの状態方程式関数
    ```matlab
        X_d = drone_sim(X,T)
        %% 出力：
        %%      X_d ： 状態ベクトルの時間微分
        %% 入力：
        %%      X ： ドローンの状態ベクトル
        %%          （変位，姿勢角，およびそれらの微分）
        %%      T ： 各ロータ推力をまとめたベクトル
    ```
