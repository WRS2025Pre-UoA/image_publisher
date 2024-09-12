# image_publisher
画像をpublishするだけのROS2ノード

## 環境

- Python3
- Open-CV
- cv_bridge

## 初期化
ワークスペースの`src`ディレクトリ内にリポジトリをクローンする
```bash
cd src/
git clone git@github.com:WRS2025Pre-UoA/image_publisher.git
```

## 実行方法
ワークスペースのルートディレクトリでビルド(`colcon build`)をして、ros2 runを行う  

オプション
- `path`(必須): 画像ファイルのパス
- `topic`: Publishする際のトピック名(default: "/image")
- `format`: Publishする際の画像の色形式で`color`または`mono`を指定(default: "color")

```bash
ros2 run image_publisher image_publisher --ros-args -p path:=image_path/image.png -p topic:=/qr_image
```
