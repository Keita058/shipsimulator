# UnityとROS2を用いた船舶操縦シミュレータ

## 動作確認環境

- Unity
  - Editor Ver: 2021.3.22f1
  - template: 3D(URP)
- ROS2 Humble
- Ubuntu 22.04LTS

## 使用方法
### ゲームコントローラを使って操縦する場合

- shipsimulator_Linux->ship1_view->shipsimulator_ship1_Linux.x86_64からUnityを起動
- ゲームコントローラをPC端末に接続
- ROS2のノードを起動
  - ROS-TCP-Endpoint、controller、actuator、model、sensorの各ノードを起動
  - ゲームコントローラの操作を読み取るためにjoy_linuxノードも起動
  - 必要に応じてdisturbanceノードやwaypointノードも起動してください
- ゲームコントローラの操作によって船舶の操縦が可能
  - R2ボタンでプロペラ回転数増加(加速)、L2ボタンでプロペラ回転数減少(減速)
  - 左スティックを左右に傾けることで舵角を操作可能

### 制御システムを使用する場合

- shipsimulator_Linux->ship1_view->shipsimulator_ship1_Linux.x86_64から

## 各フォルダの説明

- shipsimulator_Linux
  - Unityアプリの起動ファイルがあります。
  - ship1_view->shipsimulator_ship1_Linux.x86_64 を実行することでship1視点のシーンを展開する
- src
  - ROS-TCP-Endpoint
  - actuator_module: 操作量の変化率が一定範囲内に収まるように調整するノード
  - disturbance_module: 外乱の大きさを設定するノード
  - guidance_module: ウェイポイントの情報から船舶の制御に必要な情報を出力するノード
  - mmg_module
    - controllerノード: 船舶の操作量の決定を行うノード
    - modelノード: 操作量を基に船体運動の計算を行うノード
  - sensor_module: シミュレータ内の船舶の位置・方位、船速メッセージの受信
  - shipsim_msgs_module: 独自メッセージが入っているモジュール
  - waypoint_module: 船舶が通過するウェイポイントの情報を出力するノード
- simulator_project.unitypackage: Unityプロジェクトの入ったパッケージ

## Unityの準備

- Unity Hubを[公式サイト](https://unity.com/ja/download)からダウンロード
- Unity Hubを起動後、エディターをインストールする
  - エディターバージョン**2021.3.22f1**をインストールしてください
- 「New Project」を押し、エディターバージョンは **2021.3.22f1** を選択、テンプレートは **3D(URP)** を選択後、「Create Project」を押す
- Unityのメニュー「Window → Package Manager」を開き、「+」をクリック、「Add package from git URL...」を選択し、以下のURLを入力後「Add」ボタンを押す
    ```
    https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector#v0.7.0
    ```
- メニューに「Robotics」が追加されるのでそれを開き、「ROS Settings」の「Protocol」の選択を「ROS2」に変更
- Unityのメニュー「Asset→Import Package→Custom Package」を選択し、cloneしたレポジトリ内の「Unity_shipsimulator_project.unitypackage」を選択する


## ROS2の準備

- ROS humbleのインストール
- joy_linuxパッケージのインストール
- ROS-TCPに関するパッケージのインストール
