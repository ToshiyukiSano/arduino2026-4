// dai4_ai_remote（第4回：手話・ポーズをAIで見分けて、Arduino を操作する）
// 【教材・実機未検証】
//
// ボード：Arduino UNO R4 WiFi（UNO R3・R4 Minima では動きません）
// 必要なライブラリ：Servo、ArduinoBLE
//   （UNO R4 WiFi の通信用モジュールのファームウェアが 0.2.0 以上であること）
//
// 配線（基本）：
//   ステップ1：外付けLED 1個　D2 → 抵抗 → LED の＋（長い足）、LED の−（短い足）→ GND
//   ステップ2：サーボ　信号 → D9、電源（＋）→ 5V、GND（−）→ GND（第3回と同じ）
//   本体の LED マトリクス：届いた指示の記号を表示する（配線なし）
// 配線（発展・時間が余った人）：
//   LED を追加　D3・D4・D5・D6（それぞれ 抵抗 → LED → GND）
//
// 操作指示は5種類。PC（Chrome で開いた index.html）の AI がカメラの画像を見分け、
// クラス名に合わせて1文字を Bluetooth（BLE）で送る：
//     クラス「点灯」→ O：LED を点灯（サーボはそのまま）
//     クラス「消灯」→ F：LED を消灯（サーボはそのまま）
//     クラス「右」　→ R：サーボを右へ（LED はそのまま）
//     クラス「中央」→ C：サーボを中央へ（LED はそのまま）
//     クラス「左」　→ L：サーボを左へ（LED はそのまま）
//     クラス「待機」→ 何も送らない（LED もサーボも、直前の状態のまま）
//   届いた指示は、LED マトリクスに記号で表示する。
//   AI は PC の中で動いています。Arduino が受け取るのは1文字だけです。
//   書き込むときは PC と USB でつなぐ（書き込みと電源）。
//   書き込んだあとは PC の USB を外し、USB 充電器などから Arduino の USB 端子へ給電する。
//
// サーボの向き：サーボを軸（ホーン）が上になるように置き、上から見る。
//   多くのサーボは角度の数字が大きいほど反時計回り（左回り）になるので、
//   初めの値は 右＝0°、中央＝90°、左＝180°。
//   実物で左右が逆に動くときは、ANGLE_RIGHT と ANGLE_LEFT の数字を入れかえる。
//
// 生徒が変えるのは「★ここを変える」の中だけです（発展では handleCommand に1行足します）。

#include <Servo.h>               // サーボを動かすための機能を読み込む
#include <ArduinoBLE.h>          // Bluetooth（BLE）を使うための機能を読み込む
#include "Arduino_LED_Matrix.h"  // 本体の LED マトリクスを使うための機能を読み込む

// LED のピン（ここは変えない）
const int RED    = 2;   // 基本の LED（ステップ1）
const int YELLOW = 3;   // 発展
const int BLUE   = 4;   // 発展
const int GREEN  = 5;   // 発展
const int WHITE  = 6;   // 発展

// =================== ★ここを変える（ここから）===================

// ① 機器名：先生から割り当てられた自分の番号にする（NGS-01 〜 NGS-11）
const char DEVICE_NAME[] = "NGS-01";

// ② サーボの角度（0〜180）。左右が逆に動くときは、右と左の数字を入れかえる
const int ANGLE_RIGHT  = 0;    // 右
const int ANGLE_CENTER = 90;   // 中央
const int ANGLE_LEFT   = 180;  // 左

// ③ 届いた指示を LED マトリクスに表示する記号（横12 × 縦8。「#」が光る、「.」が消える）
//    1行は必ず12文字。行は8行。余裕があれば、自分の記号に描きかえてよい
const char* PICTURE_ON[8] = {      // 点灯：光っている丸
  "....####....",
  "..########..",
  ".##########.",
  ".##########.",
  ".##########.",
  ".##########.",
  "..########..",
  "....####...."
};

const char* PICTURE_OFF[8] = {     // 消灯：中が空の丸
  "....####....",
  "..##....##..",
  ".#........#.",
  ".#........#.",
  ".#........#.",
  ".#........#.",
  "..##....##..",
  "....####...."
};

const char* PICTURE_RIGHT[8] = {   // 右：右向きの矢印
  "............",
  ".......#....",
  ".......##...",
  ".##########.",
  ".##########.",
  ".......##...",
  ".......#....",
  "............"
};

const char* PICTURE_CENTER[8] = {  // 中央：上向きの矢印
  ".....##.....",
  "....####....",
  "...######...",
  "..##.##.##..",
  ".....##.....",
  ".....##.....",
  ".....##.....",
  "............"
};

const char* PICTURE_LEFT[8] = {    // 左：左向きの矢印
  "............",
  "....#.......",
  "...##.......",
  ".##########.",
  ".##########.",
  "...##.......",
  "....#.......",
  "............"
};

// =================== ★ここを変える（ここまで）===================

static_assert(ANGLE_RIGHT >= 0 && ANGLE_RIGHT <= 180, "ANGLE_RIGHT は 0〜180 にしてください");
static_assert(ANGLE_CENTER >= 0 && ANGLE_CENTER <= 180, "ANGLE_CENTER は 0〜180 にしてください");
static_assert(ANGLE_LEFT >= 0 && ANGLE_LEFT <= 180, "ANGLE_LEFT は 0〜180 にしてください");

// ---- ここから下は、発展で handleCommand に1行足すとき以外は変更しない ----
const int SERVO_PIN = 9;  // サーボの信号の線（D9）
const int LED_PINS[5] = {RED, YELLOW, BLUE, GREEN, WHITE};

// BLE の設定（index.html と同じ番号。第3回とは別の番号）
BLEService remoteService("2fb61ef3-3619-447d-8e6c-58a330ada446");
BLEByteCharacteristic commandChar("a757a3b8-787c-44bb-a768-95c597baf42b",
                                  BLEWrite | BLEWriteWithoutResponse);
BLEStringCharacteristic statusChar("67fc9eee-e535-4917-99f6-222dc95351ce",
                                   BLERead | BLENotify, 32);

Servo servo;
ArduinoLEDMatrix matrix;
bool bleReady = false;
char lastCommand = '-';   // まだ指示を受け取っていない
int lastAngle = 90;
bool redOn = false;
char pendingCmd = 0;      // Bluetooth で届いた指示（loop の中で動かす）

// つながったときの絵（小さなチェック）
const char* PICTURE_READY[8] = {
  "............", "............", "..........#.", ".........#..",
  "..#.....#...", "...#...#....", "....#.#.....", ".....#......"
};

// 数字（横3 × 縦5）：待っている間、機器名の番号を LED マトリクスに表示する
const char* DIGITS[10][5] = {
  {"###", "#.#", "#.#", "#.#", "###"}, {".#.", "##.", ".#.", ".#.", "###"},
  {"###", "..#", "###", "#..", "###"}, {"###", "..#", "###", "..#", "###"},
  {"#.#", "#.#", "###", "..#", "..#"}, {"###", "#..", "###", "..#", "###"},
  {"###", "#..", "###", "#.#", "###"}, {"###", "..#", "..#", "..#", "..#"},
  {"###", "#.#", "###", "#.#", "###"}, {"###", "#.#", "###", "..#", "###"}
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

// 外付け LED をすべて消す
void ledsOff() {
  for (int i = 0; i < 5; i++) digitalWrite(LED_PINS[i], LOW);
  redOn = false;
}

// 発展：指定した LED だけを光らせる（ほかは消す）
void ledOnly(int pin) {
  ledsOff();
  digitalWrite(pin, HIGH);
  redOn = (pin == RED);
}

// 発展：LED を D2 → D6 の順に1回ずつ光らせる
void ledSequence() {
  for (int i = 0; i < 5; i++) {
    ledOnly(LED_PINS[i]);
    delay(200);
    BLE.poll();  // 光らせている間も Bluetooth の処理を続ける
  }
  ledsOff();
}

// PC へ今の状態を知らせる（例：「R:1:0:NGS-01」＝最後の指示:LED（1＝点灯）:角度:機器名）
void report() {
  char text[32];
  snprintf(text, sizeof(text), "%c:%d:%d:%s", lastCommand, redOn ? 1 : 0, lastAngle, DEVICE_NAME);
  Serial.println(text);
  if (bleReady) statusChar.writeValue(text);
}

// 指示を受け取ったときの動き
//   点灯・消灯ではサーボを動かさない。右・中央・左では LED を変えない。
void handleCommand(char cmd) {
  if (cmd == 'O') {               // 点灯：LED をつける
    digitalWrite(RED, HIGH);
    redOn = true;
    showPicture(PICTURE_ON);
  } else if (cmd == 'F') {        // 消灯：LED を消す
    digitalWrite(RED, LOW);
    redOn = false;
    showPicture(PICTURE_OFF);
  } else if (cmd == 'R') {        // 右：サーボを右へ
    servo.write(ANGLE_RIGHT);
    lastAngle = ANGLE_RIGHT;
    showPicture(PICTURE_RIGHT);
  } else if (cmd == 'C') {        // 中央：サーボを中央へ
    servo.write(ANGLE_CENTER);
    lastAngle = ANGLE_CENTER;
    showPicture(PICTURE_CENTER);
  } else if (cmd == 'L') {        // 左：サーボを左へ
    servo.write(ANGLE_LEFT);
    lastAngle = ANGLE_LEFT;
    showPicture(PICTURE_LEFT);
  } else {
    return;  // それ以外（改行など）は何もしない
  }
  lastCommand = cmd;
  report();
}

// Bluetooth で指示が届いたとき
void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
  pendingCmd = (char)commandChar.value();  // ここでは受け取るだけ。動かすのは loop の中
}

// PC とつながったとき
void onConnected(BLEDevice central) {
  Serial.print("connected: ");
  Serial.println(central.address());
  showPicture(PICTURE_READY);  // つながったらチェックの絵
  report();
}

// PC との接続が切れたとき
void onDisconnected(BLEDevice central) {
  Serial.println("disconnected");
  showNumber();     // 番号の表示に戻す（LED とサーボはそのまま）
  BLE.advertise();  // もう一度つなげるように待つ
}

void setup() {
  Serial.begin(9600);
  matrix.begin();

  for (int i = 0; i < 5; i++) pinMode(LED_PINS[i], OUTPUT);
  ledsOff();

  servo.attach(SERVO_PIN);    // サーボの信号を出すピン（D9）
  servo.write(ANGLE_CENTER);  // 電源を入れたら中央へ
  lastAngle = ANGLE_CENTER;

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
    Serial.println("BLE start failed. Serial Monitor (9600): O F R C L");
  }
}

void loop() {
  if (bleReady) BLE.poll();  // Bluetooth の処理（止めずに毎回まわす）

  if (pendingCmd != 0) {     // Bluetooth で指示が届いていたら動かす
    char cmd = pendingCmd;
    pendingCmd = 0;
    handleCommand(cmd);
  }

  // 予備：シリアルモニター（9600）から O・F・R・C・L を送っても動く
  while (Serial.available() > 0) {
    handleCommand((char)Serial.read());
  }
}
