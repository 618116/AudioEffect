# 課題の説明
Phase Vocoderを使ったリアルタイムタイムストレッチ

# ビルド環境
Emscripten 5.0.2
Windows 11

# デモ動作環境
Google Chrome 145.0.7632.110(Windows)
AudioWorkletが動作できる環境(localhostかhttpsが必要)

# 生成AIを使用した箇所
補助ライブラリー（リングバッファ・FFT）
デモアプリケーションのhtml/JavaScriptレイヤー
デモアプリケーションの音楽データ
TimeStretcherクラスの作成・PhaseVocoderクラスのひな型を作成
そのほかデバッグ・アルゴリズムの検証

# アルゴリズムの説明

## 処理概要
1. 入力信号をリングバッファに格納し、Hann窓を適用してFFTを実行 
2. 各binに対してPhase Vocoderの（スペクトラムにおける）水平位相更新を実行
3. 周波数方向の位相整合（垂直位相補正）を適用  
4. IFFT後、窓掛けとOLAで時間領域へ再合成  

## 垂直位相補正仕様

### 目的
- 隣接bin間の位相関係を保持するphase lockingの一種  
- ピーク追跡やbin分類を行わず、全binに同一ルールを適用
- リアルタイム処理を念頭に、コスパのいい処理を行う

### 処理手順
1. 位相勾配の抽出
   元スペクトル \(X[k]\) から隣接bin間の位相勾配を単位ベクトルとして計算する。これは時間領域における群遅延になる。
   \[
   V[k] = \frac{X[k]\cdot X^*[k-1]}{|X[k]\cdot X^*[k-1]|}
   \]

2. 隣接bin提案の生成
   水平更新後スペクトル \(H[k]\) から、上下隣接binの提案を位相勾配で回転して生成する。  
   \[
   P_{below}=H[k-1]\cdot V[k],\quad
   P_{above}=H[k+1]\cdot V^*[k+1]
   \]

3. 合意ベクトルの算出 
   自binと隣接提案を重み付きで加算する。
   \[
   C[k] = H[k] + w_b P_{below} + w_a P_{above}
   \]
   \(w_b,w_a\) if分を使わずに終端のbinを処理するためのマスク

4. 位相反映（振幅保持） 
   合意ベクトルを正規化して位相のみ採用し、元振幅 \(|X[k]|\) を再適用する。  
   \[
   Y[k] = |X[k]|\cdot \frac{C[k]}{|C[k]|}
   \]

### 実装上の特性
- 終端のbinの処理は分岐ではなく index/mask で処理することでSIMD/コンパイラ最適化が聞きやすくする 
- ピーク検出を行わずに全binに対して処理を行うことで、複雑さを減らしている

### Limitation & Todo
- 左右のbinとしか同期しないので、エネルギーが複数binにわたる場合問題になるかもしれない
    →位相勾配を求めるbinの範囲を広げる
- 強いトランジェントは完ぺきに処理できない
    →Phase resetなど別のadhocが必要
- FFTサイズ・Overlap範囲などの調整

# その他
SignalSmithのアプローチ（https://signalsmith-audio.co.uk/writing/2023/stretch-design/）から着想を得ています。
彼のtwo-step sweepはIIRを使ったものですが、それをFIRにすることが最初の着眼点です。 