ECG Transmitter for Raspberry pi pico
====

心電信号測定器

## Description
心電信号をADで取得してOLEDに心電図として表示出来ます。
取得したデータはリアルタイムにUSBシリアル(115200bps)で送信します。
シリアルプロッタ等を用意することで、PCでも心電図表示やデータ記録を行うこともできます。

解説:  http://sousuch.web.fc2.com/DIY/ecg/

## Example
![WIREING DIAGRAM](https://github.com/sousu-ch/ECG_Transmitter/blob/master/sokutei.jpg "photo")
![WIREING DIAGRAM](https://github.com/sousu-ch/ECG_Transmitter/blob/master/block.gif "block diagram")
![WIREING DIAGRAM](https://github.com/sousu-ch/ECG_Transmitter/blob/master/circuit.jpg "circuit")

## Requirement
EARLEPHILHOWER_PICOを使用しています

## Usage
電源ONで動作開始し、測定電極からの心電波形および心拍数を、OLEDに表示します。

操作部はボタン1か所で、ボタン押下ごとに表示速度を4段階に変更できます。
例えば、0だと約1心拍、3だと約4心拍になります。
表示速度の基本周期は心拍数の4回平均から算出しています。
このため、心拍表示が安定してからボタンを押すと、表示と心拍が同期しやすいです。

心拍が安定して取得でき多々同課は、心拍検出マークや心拍数を目安にするといいです。

心電波形および心拍数は、USBシリアルで送信されます。
115200bpsでCSV形式です。
...

## Install
EARLEPHILHOWER_PICOをインストールしてから、ビルド書き込みしてください。

## Licence
The source code is licensed MIT.
