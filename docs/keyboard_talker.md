## KeyBoardTalkerノード 概要

<!-- 
1. 内部に変数もちます
    1.1. それぞれこんな役目です
2. キーボードの値とりますよー
3. 
 -->
KeyBoardTalkerノードの機能は、ユーザからのキーボード操作を解釈し、The Rooterの状態を決定することである。
The Routerの機能を実現するために管理するべき状態は、1. 現在の操作情報, 2. 手動モードか自動モードか, 3. 自動モード時に逆走するか順走するか、3つである。
この3つの状態を表現するために、ノードのインスタンス内に表に示す6つの変数を保持する。

directionとcurrent_gearは操作情報に相当し、ユーザによる入力が0.5秒ごとに反映される。
前者は進行方向を保持する変数であり、'forward', 'backward', 'right', 'left', 'stop', 'immd_stop', Noneの5つの値を取る。
それぞれ、前進、後退、右旋回、左旋回、停止、緊急停止、不定値に対応する。
停止は、台形制御による緩やかな停止、緊急停止は、そのときの速度によらず即座に停止することを表す。
後者は前後方向での移動速度を表し、1から4までの整数値を取りうる。
この変数は数字が小さければ進行速度が遅く、大きければ速い状態を示す。

manual_modeとauto_modeは手動モードか自動モードかを表す。
どちらもbool型であり、つねに1つのみがTrueである。
すなわちどちらもTrue、もしくはどちらもFalseとはならず、状態数は2つである。
manual_modeがTrueのときは手動モード、auto_modeがTrueのときは自動モードを表す。

forward_auto, backward_autoは自動モード時での振る舞いを規定する。
この2変数も排他的であり、どちらか1つのみがTrueである。
forward_autoがTrueのときは自動モードの際に走行経路を順走し、
backward_autoがTrueのときは逆走する

<!-- そのため、1変数でも表現できるが、構想段階では、3つ目のモードも追加する可能性があったため、
拡張性を優先し、複数の変数に分けた。 -->

| 変数名        | 型        | 対応する機能                               | デフォルト値 |
|:--------------|:----------|:--------------------------------------|:--------|
| direction     | str\|None | 進行方向を保持する('forward', 'backward'など) | None    |
| current_gear  | int       | 速度を保持する(1,2,3,4)                     | 1       |
| manual_mode   | bool      | 手動モードか否か                              | True    |
| auto_mode     | bool      | 自動モードか否か                              | False   |
| forward_auto  | bool      | 自動モード時に順走するか                        | False   |
| backward_auto | bool      | 自動モード時に逆走するか                        | True    |



以上の状態変数はユーザによるキーボード操作によって決定される。
キー操作と変更される変数の関係を次の表に示す。
ただし加速操作については、すでに上限速度に達している場合にはその操作は無視される。減速についても同様である。

| キー操作 | 機能               | 変数                        | 値          |
|:------:|:-------------------|:----------------------------|:------------|
|   w    | 前進               | direction                   | 'forward'   |
|   s    | 後退               | direction                   | 'backward'  |
|   a    | 左旋回             | direction                   | 'left'      |
|   d    | 右旋回             | direction                   | 'right'     |
|   q    | 緊急停止           | direction                   | 'immd_stop' |
|   j    | 加速               | current_gear                | インクリメント     |
|   h    | 減速               | current_gear                | デクリメント      |
|   o    | マニュアルモード           | auto_mode, manual_mode      | False, True |
|   p    | 自動モード            | auto_mode, manual_mode      | True, False |
|   u    | 再走モード(自動モード時) | forward_mode, backward_mode | True, False |
|   i    | 逆走モード(自動モード時) | forward_mode, backward_mode | False, True |
| それ以外 | 停止               | direction                   | 'stop'      |

以上に述べた状態管理用の変数を、キーボード操作が行われるごとに
後述するJSON(JavaScript Object Notation)オブジェクトに変換してトピック/keyboard_controlとして発行している。

## /keyboard_controlトピックの仕様

本トピックは、状態変数の変数名をキーに、対応する値をバリューとしたJSONオブジェクトを文字列にダンプし、
ROS2組み込みのString型のデータとして発行している。
たとえば、以下のようなメッセージである。なお、見やすさのために改行やインデントを含んでいるが実際には1行で表現されている。

```json
{ 
  "direction": "forward",
  "speed": 1,
  "manual_mode": true,
  "auto_mode": false,
  "forward_auto": false,
  "backward_auto": true
}
```

各値の型は、上述したインスタンス内部の状態変数の型に対応するJSONオブジェクトの型である。


## 実装上の工夫点
本ノードとトピックの実装上の工夫点は、トピック内のデータ構造としてJSONを採用した点である。
JSONはJavaScriptやPythonのみならず、さまざまなプログラミング言語や通信路に採用されている、
テキストによるデータ構造のシリアライズ形式である。
YAMLやTOMLと異なり、Pythonの標準モジュールにPythonのデータ構造と相互に変換する関数が含まれる点、
XMLに比較して解析が軽量な点などの複数のメリットがある。
