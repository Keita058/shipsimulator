# UnityとROS2を用いた船舶操縦シミュレータ

## 概要

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

## シミュレータの起動
- ゲームコントローラの接続
- 各ノードの起動方法
- Unityの起動方法

## 操縦方法
- ゲームコントローラによって舵角(方位)とプロペラ回転数(速度)を操作できる

### 舵角の変更

### プロペラ回転数の変更