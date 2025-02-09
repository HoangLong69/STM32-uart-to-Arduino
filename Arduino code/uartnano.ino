const int buttonPin = 2;  // Chân kết nối nút nhấn
bool buttonState = LOW;   // Trạng thái hiện tại của nút nhấn
bool lastButtonState = LOW; // Trạng thái trước đó của nút nhấn
int MQ2_Value = 0;         // Giá trị nhận từ STM32
const int ledPin = 13; 

void setup() {
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);  // Khởi tạo UART với tốc độ 9600 baud
}

void loop() {
  buttonState = digitalRead(buttonPin);

  // Kiểm tra nếu nút nhấn được nhấn
  if (buttonState == HIGH && lastButtonState == LOW) {
    Serial.write('1');  
    delay(100);  
  }

  lastButtonState = buttonState;

  // Kiểm tra nếu có dữ liệu từ STM32 (giá trị MQ2)
  if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n'); 
    Serial.print("");
    Serial.println(receivedData); 
  }

  delay(10);
}