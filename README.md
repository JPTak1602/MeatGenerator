# MeatGenerator
Meat image generator for meat judger but prottype

これは、ミートジャッジングの練習用の、胸最長筋のような模様を出力するプログラムです。

楕円面積、色、白い点の数と大きさのそれぞれに一定の範囲内で乱数を与え、描画しているだけです。
白い点を脂肪交雑、とみなしてほしいのですが、これはフィルタを使用して脂肪交雑っぽくしようとしています。
本当はチューリングパターンを用いていい感じの時間発展のところで止めて脂肪交雑っぽくしよう、という計画でしたがまったくこのフィルタは誤っているので、一応動いているのは9割9分バグです。
これから修正予定ですが、練習には使用できなくもない、と思ったので一応公開します。

使い方は自由ですが、できた模様を見比べて目利きの練習に使用してください。

答え合わせは、下に出力しているBCS、Size、BMSを参考にしてください。

BCSは一応日本食肉格付け協会のBCS1－8までの範囲でランダムに割り振っていますのでおおよそスタンダードに近い色が出ているのではないかと思います。

Sizeは楕円の面積を計算しているだけです。

BMSは楕円の面積と白く塗りつぶされている範囲の面積を算出して、口田先生の式（1999）を使用して計算しています。
完全にランダムなので、ありえないようなBMSの値を出示すことがありますが、ご容赦ください。
あくまで練習用なので...

ソースについては、もうひどいのでスルーでお願いします。
眠い朝方に書いた、ごちゃごちゃなものをリファクタリングしようしようと思いつつ、手を付けていないのです。怠惰ですね。
「とりあえず動くやつ」という、アレなものです...

質問や不具合があれば対応しますので連絡をください。

/*
Tatsuki KAMATA
Graduate school of Agriculture,　Hokkaido University