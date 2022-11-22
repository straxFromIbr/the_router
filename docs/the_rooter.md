# The Rooterの仕様

上述したThe Routerの機能を満たすために、以下の図のようにKeyBoardTalker,
MainController, TrapeControllerの3つのノードを作成した。

KeyBoardTalkerはユーザの入力に基づいてThe Routerの状態を決定し、その情報を/keyboard_topicとして発行する。
MainControllerは、/keyboard_topicに基づいて、The Routerの進行方向を決定し、
その情報をトピック/control_topicとして発行する。

TrapeControllerはcontrol_topicの進行方向情報から、モータの台形制御を行うためのパラメータを計算する。
パラメータはRaspberry Pi Mouse用のトピック/cmd_velを通じてモータに伝えられる。

以下に各ノードの詳細について記述する。