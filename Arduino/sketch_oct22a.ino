#include <math.h>
#include <stdlib.h>

#define NUM_MOTORS 6
int Step_pins[NUM_MOTORS] = {A0, A6, 46, 26, 36, 23};  // Chân STEP mới
int Dir_pins[NUM_MOTORS] = {A1, A7, 48, 28, 34, 25};   // Chân DIR mới
int Enable_pins[NUM_MOTORS] = {38, A2, A8, 24, 30, 27}; // Chân ENABLE mới

#define HOMING_SPEED 4  // Tốc độ cho homing (giảm nếu cần)
#define BACKOFF_DISTANCE_MM 2  // Khoảng cách lùi lại sau khi gặp endstop (mm)

// Định nghĩa các chân endstop
int Endstop_pins[4] = {3, 14, 15, 18}; // Endstop cho trục X, Y, Z

#define Pump_L 8
#define Pump_R 9

#define RELAY_PIN 29  // Chân điều khiển relay
#define RELAY_ON_DURATION 500  // Thời gian bật relay (0.5 giây)

bool relayActivated = false;  // Biến để theo dõi relay đã bật hay chưa
unsigned long relayStartTime = 0;  // Lưu thời điểm bắt đầu bật relay
bool motorsAtTarget = false;  // Biến theo dõi trạng thái đạt đến vị trí mong muốn
bool skipFirstTime = true;  // Biến để bỏ qua lần đầu tiên
bool relayCommandSent = false;  // Biến để theo dõi đã gửi lệnh relay on

unsigned long startTime = 0;
unsigned long runDuration = 0;
bool motorRunning = false;

unsigned long previousMillis = 0;  // Lưu thời gian lần gửi trước đó
const long interval = 500;         // Thời gian giữa các lần gửi (0.5 giây)

boolean en = false;
const byte numChars = 100;
char receivedChars[numChars];
boolean newData = false;
float positions[6] = {0};
long curSpeed = 2;
long steps[6] = {0};

// Motor control variables
int8_t Dir_M[NUM_MOTORS] = {0};
volatile int16_t Count_timer[NUM_MOTORS] = {0};
volatile long Steps[NUM_MOTORS] = {0};
int16_t Count_TOP[NUM_MOTORS] = {0}, Count_BOT[NUM_MOTORS] = {0};
int8_t Microstep = 4;
unsigned int DK_BX = 145;
long lastStep[NUM_MOTORS] = {0};
const int stepPrintInterval = 50;


void pin_INI() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(Enable_pins[i], OUTPUT);
    pinMode(Step_pins[i], OUTPUT);
    pinMode(Dir_pins[i], OUTPUT);
    digitalWrite(Enable_pins[i], LOW);
  }

  for (int i = 0; i < 3; i++) {
    pinMode(Endstop_pins[i], INPUT_PULLUP);
  }

  pinMode(RELAY_PIN, OUTPUT);  // Khai báo relay pin là OUTPUT
  pinMode(Pump_L, OUTPUT);
  pinMode(Pump_R, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Đặt relay ở trạng thái tắt
  digitalWrite(Pump_L, LOW);  // Đặt relay ở trạng thái tắt
  digitalWrite(Pump_R, LOW);  // Đặt relay ở trạng thái tắt
}

void timer_INI() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12); 
  OCR1A = (F_CPU / (8 * 10000UL)) - 1;
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

ISR(TIMER1_COMPA_vect) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (Dir_M[i] != 0) {
      Count_timer[i]++;
      digitalWrite(Step_pins[i], Count_timer[i] <= Count_TOP[i] ? HIGH : LOW);
      if (Count_timer[i] > Count_BOT[i]) {
        Count_timer[i] = 0;
        Steps[i] += Dir_M[i];
      }
    }
  }
}

void Speed_M(int motor, long V) {
  Dir_M[motor] = V < 0 ? -1 : (V > 0 ? 1 : 0);
  digitalWrite(Dir_pins[motor], V < 0 ? HIGH : LOW);
  Count_BOT[motor] = abs(V);
  Count_TOP[motor] = abs(V) / 2;
}

void Move_M(int motor, long D, long S) {
  long Dx = D - Steps[motor];
  if (abs(Dx) == 0) {
    Speed_M(motor, 0); 
  } else {
    Speed_M(motor, Dx < 0 ? -S : S);
  }
}

void Send_Pos() {
  unsigned long A = Steps[0];
  unsigned long B = Steps[1];
  unsigned long C = Steps[2];
  unsigned long D = Steps[4];
  unsigned long E = Steps[5];

  unsigned long currentMillis = millis();

  // Kiểm tra nếu đã đạt đến thời gian để gửi giá trị
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Cập nhật thời gian lần gửi trước
    Serial.println(String(A) + " " + String(B) + " " + String(C) + " " + String(D) + " " + String(E));
  }


}

void Wheel_Control(int16_t x, int16_t y, uint32_t z) {
  int16_t speed_M1, speed_M2;
  float speed_mean;

  speed_M1 = x + y;
  speed_M2 = x - y;

  speed_mean = (1/(speed_M1*0.08) + 1/(speed_M2*0.08))*3.141592653589793*DK_BX/2;
  
  Speed_M(4,speed_M1);
  Speed_M(5,speed_M2);

  startTime = millis();
  runDuration = z/speed_mean*1000;
  digitalWrite(Pump_L, HIGH);  // Đặt relay ở trạng thái tắt
  digitalWrite(Pump_R, HIGH);  // Đặt relay ở trạng thái tắt
  motorRunning = true;
}

void checkMotorRunning() {
  if (motorRunning && (millis() - startTime >= runDuration)) {
    Speed_M(4, 0);
    Speed_M(5, 0);
    digitalWrite(Pump_L, LOW);  // Đặt relay ở trạng thái tắt
    digitalWrite(Pump_R, LOW);  // Đặt relay ở trạng thái tắt
//    Send_Pos();
    motorRunning = false;
  }
}

void homeAxesSimultaneously() {

  // Đặt động cơ chạy về hướng âm cho cả 3 trục
  for (int i = 0; i < 4; i++) {
    Speed_M(i, -HOMING_SPEED);  // Đặt tốc độ chạy về gốc cho cả 3 động cơ
  }

  bool homingComplete[4] = {false, false, false, false};  // Theo dõi trục nào đã hoàn thành homing

  // Tiếp tục chạy cho đến khi tất cả các trục đều về vị trí gốc
  while (!(homingComplete[0] && homingComplete[1] && homingComplete[2]&& homingComplete[3])) {
    for (int i = 0; i < 4; i++) {
      if (!homingComplete[i] && digitalRead(Endstop_pins[i]) == LOW) {
        // Khi trục i chạm endstop
        Speed_M(i, 0);  // Dừng động cơ
        homingComplete[i] = true;  // Đánh dấu trục i đã hoàn thành homing
      }
    }
    
  }

  // Đặt lại vị trí hiện tại là 0 cho tất cả các trục
  for (int i = 0; i < 4; i++) {
    Steps[i] = 0;
  }
}


void parseData() {
  if (!newData) return;
  newData = false;

  char* token = strtok(receivedChars, " ");
  if (token && String(token).startsWith("G10")) {
    while ((token = strtok(NULL, " "))) {
      String stringValue = String(token);
      if (stringValue.startsWith("X")) positions[0] = atof(stringValue.substring(1).c_str());
      else if (stringValue.startsWith("Y")) positions[1] = atof(stringValue.substring(1).c_str());
      else if (stringValue.startsWith("Z")) positions[2] = atof(stringValue.substring(1).c_str());
      else if (stringValue.startsWith("A")) positions[3] = atof(stringValue.substring(1).c_str());
      else if (stringValue.startsWith("B")) positions[4] = atof(stringValue.substring(1).c_str());
      else if (stringValue.startsWith("E")) positions[5] = atof(stringValue.substring(1).c_str());
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
      steps[i] = i < 3 ? positions[i] * Microstep * 200 / 8 : positions[i];
    }
  }

  if (token && String(token).startsWith("G20")) {
    while ((token = strtok(NULL, " "))) {
      String stringValue = String(token);
      if (stringValue.startsWith("F")) curSpeed = atof(stringValue.substring(1).c_str());
      else if (stringValue.startsWith("E0")) disableAll();
      else if (stringValue.startsWith("E1")) enableAll();
    }
  }
}

void SendRelayStatus(bool relayOn) {
  String command = "G20 K";
  command += relayOn ? "1" : "0";
  command += " L" + String(Steps[4]);
  command += " R" + String(Steps[5]);
  Serial.println(command);
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char rc;
  while (Serial.available() > 0 && !newData) {
    rc = Serial.read();
    if (rc != '\n') {
      receivedChars[ndx++] = rc;
      if (ndx >= numChars) ndx = numChars - 1;
    } else {
      receivedChars[ndx] = '\0';
      ndx = 0;
      newData = true;

      // Gửi lệnh relay on/off chỉ khi nhận dữ liệu từ Python
      if (relayActivated && !relayCommandSent) {
        SendRelayStatus(true);    // Gửi lệnh relay on
        relayCommandSent = true;  // Đánh dấu là đã gửi
      } else if (!relayActivated && relayCommandSent) {
        SendRelayStatus(false);   // Gửi lệnh relay off
        relayCommandSent = false; // Đặt lại để sẵn sàng gửi lệnh on lần tới
      } else {
        SendRelayStatus(false);   // Gửi lệnh relay off
      }
    }
  }
}

void checkMotorsAndActivateRelay() {
  // Kiểm tra nếu cả 3 động cơ đã đến vị trí mong muốn
  if (Steps[0] == steps[0] && Steps[1] == steps[1] && Steps[2] == steps[2]) {
    if (!motorsAtTarget) {  // Chỉ khi vừa đạt vị trí
      motorsAtTarget = true;  // Đánh dấu đã đạt vị trí mong muốn

      if (!skipFirstTime) {  // Nếu không phải lần đầu tiên
        relayStartTime = millis();  // Ghi lại thời điểm bật relay
        digitalWrite(RELAY_PIN, HIGH);  // Bật relay
        relayActivated = true;  // Đặt cờ relay đã kích hoạt
      } else {
        skipFirstTime = false;  // Bỏ qua lần đầu, lần sau sẽ kích hoạt
      }
    }
  } else {
    motorsAtTarget = false;  // Reset trạng thái khi động cơ đang di chuyển
  }

  // Kiểm tra nếu relay đã bật và đã quá thời gian RELAY_ON_DURATION
  if (relayActivated && (millis() - relayStartTime >= RELAY_ON_DURATION)) {
    digitalWrite(RELAY_PIN, LOW);  // Tắt relay sau 0.5 giây
    relayActivated = false;        // Đặt cờ relay về false để có thể kích hoạt lại lần sau
  }
}


void runSteppers() { 
  if (en) {
    Move_M(0, steps[0], curSpeed);
    Move_M(1, steps[1], curSpeed);
    Move_M(2, steps[2], curSpeed);
    Move_M(3, steps[0], curSpeed);
    if (positions[5]!=0){
      if (positions[5]==1){
        Wheel_Control(200, map(positions[3], -335, 335, -150, 150), 500);
      }
      else{
        Wheel_Control(100, map(positions[3], -335, 335, -75, 75), 500);
      }
    }
  }
}

void enableAll() {
  for (int i = 0; i < NUM_MOTORS; i++) digitalWrite(Enable_pins[i], LOW);
  en = true;
}

void disableAll() {
  for (int i = 0; i < NUM_MOTORS; i++) digitalWrite(Enable_pins[i], HIGH);
  en = false;
}

void setup() {
  Serial.begin(115200);
//  Serial.begin(115200);
  pin_INI();
  timer_INI();
  enableAll();
  homeAxesSimultaneously();
  pinMode(8,OUTPUT);
}

void loop() {
  digitalWrite(8,0);
  recvWithEndMarker();
  parseData();
  runSteppers();
  checkMotorRunning();
  checkMotorsAndActivateRelay();
}
