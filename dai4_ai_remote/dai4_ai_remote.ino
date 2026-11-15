// dai4_ai_remote（第4回：手の合図をAIで見分けて、LED・LEDマトリクス・サーボを動かす）
// 【新規作成の教材・実機未検証】
//
// ボード：Arduino UNO R4 WiFi（UNO R3・R4 Minima では動きません）
// 必要なライブラリ：Servo、ArduinoBLE
//   （UNO R4 WiFi の通信用モジュールのファームウェアが 0.2.0 以上であること）
//
// 配線：
//   外付けLED（それぞれ 抵抗 → LED の＋（長い足）、LED の−（短い足）→ GND）
//     赤 D2　黄 D3　青 D4　緑 D5　白 D6
//   サーボ：信号 → D9、電源（＋）→ 5V、GND（−）→ GND（第3回と同じ）
//   1桁7セグ（追加課題・選択）：a〜g → D7・D8・D10・D11・D12・A0・A1
//
// しくみ：
//   PC（Chrome で開いた index.html）のAIが、カメラの画像を「合図A・合図B・合図C・待機」に見分ける
//   → Bluetooth Low Energy（BLE）で1文字を送る → Arduino が LED・マトリクス・サーボを動かす。
//     A = 合図A　B = 合図B　C = 合図C　N = 待機（手を出していない）
//   AI はPCの中で動いています。Arduino が受け取るのは1文字だけです。
//   書き込むときは PC と USB でつなぐ（書き込みと電源）。
//   書き込んだあとは PC の USB を外し、USB 充電器などから Arduino の USB 端子へ給電する。
//
// 生徒が変えるのは「★ここを変える」の中だけです。

#include <Servo.h>               // サーボを動かすための機能を読み込む
#include <ArduinoBLE.h>          // Bluetooth（BLE）を使うための機能を読み込む
#include "Arduino_LED_Matrix.h"  // 本体の LED マトリクスを使うための機能を読み込む

// LED のピン（配線と同じ。ここは変えない）
const int RED    = 2;
const int YELLOW = 3;
const int BLUE   = 4;
const int GREEN  = 5;
const int WHITE  = 6;
const int NONE   = -1;  // 「光らせない」

// =================== ★ここを変える（ここから）===================

// ① 機器名：先生から割り当てられた自分の番号にする（NGS-01 〜 NGS-11）
const char DEVICE_NAME[] = "NGS-01";

// ② 合図のときに光らせる LED（RED・YELLOW・BLUE・GREEN・WHITE・NONE）
//    2個目も光らせたいときは、LED_A2 などに色を入れる（光らせないときは NONE）
const int LED_A  = RED;     // 合図A
const int LED_A2 = NONE;
const int LED_B  = BLUE;    // 合図B
const int LED_B2 = NONE;
const int LED_C  = GREEN;   // 合図C（応用）
const int LED_C2 = NONE;

// ③ 合図のときのサーボの角度（0〜180）。待機のときは、今の角度のまま
const int ANGLE_A = 0;      // 合図A
const int ANGLE_B = 180;    // 合図B
const int ANGLE_C = 90;     // 合図C（応用）

// ④ LED マトリクスの絵（横12 × 縦8。「#」が光る、「.」が消える）
//    1行は必ず12文字。行は8行。
const char* PICTURE_A[8] = {   // 合図A：数字の1
  "............",
  ".....##.....",
  "....###.....",
  ".....##.....",
  ".....##.....",
  ".....##.....",
  "....####....",
  "............"
};

const char* PICTURE_B[8] = {   // 合図B：数字の2
  "............",
  "....####....",
  "...##..##...",
  ".......##...",
  "......##....",
  ".....##.....",
  "...######...",
  "............"
};

const char* PICTURE_C[8] = {   // 合図C：数字の3（応用）
  "............",
  "...######...",
  ".......##...",
  ".....###....",
  ".......##...",
  "...##..##...",
  "....####....",
  "............"
};

const char* PICTURE_WAIT[8] = {  // 待機：点が3つ
  "............",
  "............",
  "............",
  "............",
  "..##.##.##..",
  "............",
  "............",
  "............"
};

// ⑤ 追加課題：1桁7セグLED（使うときだけ true にする）
const bool USE_7SEG = false;
const bool COMMON_ANODE = false;  // 共通アノードの7セグなら true（型番で確認）
const int DIGIT_A = 1;            // 合図Aで表示する数字（0〜9）
const int DIGIT_B = 2;            // 合図B
const int DIGIT_C = 3;            // 合図C

// =================== ★ここを変える（ここまで）===================

// ---- 数値の確かめ（範囲の外なら、書き込む前に止まって知らせる）----
static_assert(ANGLE_A >= 0 && ANGLE_A <= 180, "ANGLE_A は 0〜180 にしてください");
static_assert(ANGLE_B >= 0 && ANGLE_B <= 180, "ANGLE_B は 0〜180 にしてください");
static_assert(ANGLE_C >= 0 && ANGLE_C <= 180, "ANGLE_C は 0〜180 にしてください");
static_assert(DIGIT_A >= 0 && DIGIT_A <= 9, "DIGIT_A は 0〜9 にしてください");
static_assert(DIGIT_B >= 0 && DIGIT_B <= 9, "DIGIT_B は 0〜9 にしてください");
static_assert(DIGIT_C >= 0 && DIGIT_C <= 9, "DIGIT_C は 0〜9 にしてください");

// ---- ここから下は変更しない ----
const int SERVO_PIN = 9;  // サーボの信号の線（D9）
const int LED_PINS[5] = {RED, YELLOW, BLUE, GREEN, WHITE};
const int SEG_PINS[7] = {7, 8, 10, 11, 12, A0, A1};  // a b c d e f g

// BLE の設定（index.html と同じ番号。第3回とは別の番号）
BLEService remoteService("2fb61ef3-3619-447d-8e6c-58a330ada446");
BLEByteCharacteristic commandChar("a757a3b8-787c-44bb-a768-95c597baf42b",
                                  BLEWrite | BLEWriteWithoutResponse);
BLEStringCharacteristic statusChar("67fc9eee-e535-4917-99f6-222dc95351ce",
                                   BLERead | BLENotify, 32);

Servo servo;
ArduinoLEDMatrix matrix;
bool bleReady = false;
char lastCommand = 'N';
int lastAngle = 90;

// 数字（横3 × 縦5）：待っている間、機器名の番号を LED マトリクスに表示する
const char* DIGITS[10][5] = {
  {"###", "#.#", "#.#", "#.#", "###"}, {".#.", "##.", ".#.", ".#.", "###"},
  {"###", "..#", "###", "#..", "###"}, {"###", "..#", "###", "..#", "###"},
  {"#.#", "#.#", "###", "..#", "..#"}, {"###", "#..", "###", "..#", "###"},
  {"###", "#..", "###", "#.#", "###"}, {"###", "..#", "..#", "..#", "..#"},
  {"###", "#.#", "###", "#.#", "###"}, {"###", "#.#", "###", "..#", "###"}
};

// 7セグの光らせ方（a b c d e f g の順。1 が光る）
const uint8_t SEG_DIGITS[10][7] = {
  {1,1,1,1,1,1,0}, {0,1,1,0,0,0,0}, {1,1,0,1,1,0,1}, {1,1,1,1,0,0,1}, {0,1,1,0,0,1,1},
  {1,0,1,1,0,1,1}, {1,0,1,1,1,1,1}, {1,1,1,0,0,0,0}, {1,1,1,1,1,1,1}, {1,1,1,1,0,1,1}
};

// 絵（8行 × 12文字）を LED マトリクスに表示する
void showPicture(const char* picture[8]) {
  uint8_t frame[8][12] = {0};
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 12 && picture[y][x] != '\0'; x++) {
      if (picture[y][x] == '#') frame[y][x] = 1;
    }
  }
  matrix.renderBitmap(frame, 8, 12);
}

// 機器名の最後の2文字（番号）を LED マトリクスに表示する
void showNumber() {
  uint8_t frame[8][12] = {0};
  int len = strlen(DEVICE_NAME);
  for (int k = 0; k < 2; k++) {
    int i = len - 2 + k;
    if (i < 0) continue;
    char c = DEVICE_NAME[i];
    if (c < '0' || c > '9') continue;
    int left = (k == 0) ? 2 : 7;
    for (int y = 0; y < 5; y++) {
      for (int x = 0; x < 3; x++) {
        if (DIGITS[c - '0'][y][x] == '#') frame[y + 1][left + x] = 1;
      }
    }
  }
  matrix.renderBitmap(frame, 8, 12);
}

// 外付けLEDをすべて消す
void ledsOff() {
  for (int i = 0; i < 5; i++) digitalWrite(LED_PINS[i], LOW);
}

// 外付けLEDを光らせる（NONE なら何もしない）
void ledOn(int pin) {
  if (pin != NONE) digitalWrite(pin, HIGH);
}

// 7セグに数字を出す（digit が -1 なら消す）
void showDigit(int digit) {
  if (!USE_7SEG) return;
  for (int s = 0; s < 7; s++) {
    bool on = (digit >= 0) && SEG_DIGITS[digit][s];
    if (COMMON_ANODE) on = !on;  // 共通アノードは LOW で光る
    digitalWrite(SEG_PINS[s], on ? HIGH : LOW);
  }
}

// PC へ今の状態を知らせる（例：「A:0:NGS-01」）
void report() {
  char text[32];
  snprintf(text, sizeof(text), "%c:%d:%s", lastCommand, lastAngle, DEVICE_NAME);
  Serial.println(text);
  if (bleReady) statusChar.writeValue(text);
}

// 指示を受け取ったときの動き
void handleCommand(char cmd) {
  if (cmd >= 'a' && cmd <= 'z') cmd = cmd - 'a' + 'A';  // 小文字も受け付ける
  if (cmd == 'A') {               // A なら合図A
    ledsOff();
    ledOn(LED_A);                 // 合図Aの LED を光らせる
    ledOn(LED_A2);
    showPicture(PICTURE_A);       // 合図Aの絵を表示
    servo.write(ANGLE_A);         // 合図Aの角度へ
    lastAngle = ANGLE_A;
    showDigit(DIGIT_A);
  } else if (cmd == 'B') {        // そうでなく、B なら合図B
    ledsOff();
    ledOn(LED_B);
    ledOn(LED_B2);
    showPicture(PICTURE_B);
    servo.write(ANGLE_B);
    lastAngle = ANGLE_B;
    showDigit(DIGIT_B);
  } else if (cmd == 'C') {        // そうでなく、C なら合図C（応用）
    ledsOff();
    ledOn(LED_C);
    ledOn(LED_C2);
    showPicture(PICTURE_C);
    servo.write(ANGLE_C);
    lastAngle = ANGLE_C;
    showDigit(DIGIT_C);
  } else if (cmd == 'N') {        // そうでなく、N なら待機
    ledsOff();                    // LED を消す
    showPicture(PICTURE_WAIT);    // 待機の絵を表示
    showDigit(-1);                // サーボは今の角度のまま
  } else {
    return;  // A・B・C・N 以外（改行など）は何もしない
  }
  lastCommand = cmd;
  report();
}

// Bluetooth で指示が届いたとき
void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
  handleCommand((char)commandChar.value());
}

// PC とつながったとき
void onConnected(BLEDevice central) {
  Serial.print("connected: ");
  Serial.println(central.address());
  showPicture(PICTURE_WAIT);  // つながったら待機の絵
  report();
}

// PC との接続が切れたとき
void onDisconnected(BLEDevice central) {
  Serial.println("disconnected");
  ledsOff();
  showDigit(-1);
  showNumber();     // 番号の表示に戻す（サーボはその角度のまま）
  BLE.advertise();  // もう一度つなげるように待つ
}

void setup() {
  Serial.begin(9600);
  matrix.begin();

  for (int i = 0; i < 5; i++) pinMode(LED_PINS[i], OUTPUT);
  ledsOff();
  if (USE_7SEG) {
    for (int s = 0; s < 7; s++) pinMode(SEG_PINS[s], OUTPUT);  // A0・A1 もデジタル出力として使う
    showDigit(-1);
  }

  servo.attach(SERVO_PIN);  // サーボの信号を出すピン（D9）
  servo.write(90);          // 電源を入れたら 90° へ
  lastAngle = 90;
  lastCommand = 'N';

  if (BLE.begin()) {
    BLE.setLocalName(DEVICE_NAME);
    BLE.setDeviceName(DEVICE_NAME);
    BLE.setAdvertisedService(remoteService);
    remoteService.addCharacteristic(commandChar);
    remoteService.addCharacteristic(statusChar);
    BLE.addService(remoteService);
    commandChar.setEventHandler(BLEWritten, onCommandWritten);
    BLE.setEventHandler(BLEConnected, onConnected);
    BLE.setEventHandler(BLEDisconnected, onDisconnected);
    bleReady = true;
    report();
    BLE.advertise();
    showNumber();  // 番号が出たら「接続を待っている」
    Serial.print("BLE OK: ");
    Serial.println(DEVICE_NAME);
  } else {
    // Bluetooth を始められなかった：LED マトリクスに「×」を出す
    const char* PICTURE_ERROR[8] = {
      "............", "...#....#...", "....#..#....", ".....##.....",
      ".....##.....", "....#..#....", "...#....#...", "............"
    };
    showPicture(PICTURE_ERROR);
    Serial.println("BLE start failed. Serial Monitor (9600): A / B / C / N");
  }
}

void loop() {
  if (bleReady) BLE.poll();  // Bluetooth の処理（止めずに毎回まわす）

  // 予備：シリアルモニター（9600）から A・B・C・N を送っても動く
  while (Serial.available() > 0) {
    handleCommand((char)Serial.read());
  }
}
