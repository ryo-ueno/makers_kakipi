void setup(){
  // シリアルポートを9600 bps[ビット/秒]で初期化
  Serial.begin(9600);
}

void loop(){
  // 文字列を送信
  Serial.write("1111");
  // 20ms待機
  delay(20);
}
