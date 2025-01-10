#include <AccelStepper.h>
#include <math.h>

const int stepX = 2;
const int dirX = 5;
const int stepY = 3;
const int dirY = 6;
const int stepZ = 4;  // Chân điều khiển cho động cơ Z
const int dirZ = 7;   // Chân điều khiển hướng cho động cơ Z
//const int enaPin = 8;
const int limitSwitchPinX = 13;
const int limitSwitchPinY = 11;
const int limitSwitchPinZ = 10;
//const int Namcham =9;
#define IN1 9   // Chân IN1 kết nối với D9
#define IN2 12  // Chân IN2 kết nối với D12
float ThetaX, ThetaY, ThetaZ;

String data ="";

// Tạo đối tượng AccelStepper cho 2 động cơ (kiểu DRIVER)
AccelStepper stepperX(AccelStepper::DRIVER, stepX,dirX);
AccelStepper stepperY(AccelStepper::DRIVER, stepY,dirY);
AccelStepper stepperZ(AccelStepper::DRIVER, stepZ, dirZ);

// số xung để quay góc 1 độ cho trục X và trục Y,Z
const float pulsesPerDegree_X = 24000.0 / 360.0;
const float pulsesPerDegree_YZ = 19200.0 / 360.0;


struct CubicCoefficients {
    double a0, a1, a2, a3;
};


// Hàm tính hệ số phương trình bậc 3
CubicCoefficients calculateCubic(double p0, double pT, double v0, double vT, double T) {
    CubicCoefficients coeffs;
    coeffs.a0 = p0;
    coeffs.a1 = v0;
    coeffs.a2 = (3 * (pT - p0) / (T * T)) - ((2 * v0 + vT) / T);
    coeffs.a3 = (-2 * (pT - p0) / (T * T * T)) + ((v0 + vT) / (T * T));
    return coeffs;
}

CubicCoefficients coeffX, coeffY, coeffZ;
// Hàm tính phương trình bậc 3 cho Arduino
void calculateTrajectory(double x0, double y0, double z0, double xT, double yT, double zT, double v0, double vT, double T) {
    // Tính toán hệ số cho từng trục
    coeffX = calculateCubic(x0, xT, v0, vT, T);
    coeffY = calculateCubic(y0, yT, v0, vT, T);
    coeffZ = calculateCubic(z0, zT, v0, vT, T);
}


void setup() {
    Serial.begin(115200);
  // Cấu hình chân công tắc hành trình là đầu vào
  pinMode(limitSwitchPinX, INPUT_PULLUP); 
  pinMode(limitSwitchPinY, INPUT_PULLUP); 
  pinMode(limitSwitchPinZ, INPUT_PULLUP); 
  // Cấu hình chân enable là output
//  pinMode(enaPin, OUTPUT);
  //pinMode(Namcham,OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //digitalWrite(enaPin, LOW); // Kích hoạt động cơ (LOW = ON cho CNC Shield)
  // Thiết lập tốc độ và gia tốc cho động cơ bước
  stepperX.setMaxSpeed(10000);    // Tốc độ tối đa cho stepper
  stepperX.setAcceleration(10000);// Gia tốc cho stepper
  stepperY.setMaxSpeed(15000);    
  stepperY.setAcceleration(15000); 
  stepperZ.setMaxSpeed(10000);    
  stepperZ.setAcceleration(10000); 

}

void loop() {
    
  while(Serial.available() > 0) {
     data = Serial.readStringUntil('\n'); // Đọc dữ liệu đến ký tự xuống dòng
    // Xử lý dữ liệu nhận được
  }
    if (data.length() > 0) {
      char Chudau = data.charAt(0);
      // Loại bỏ các ký tự không phải số
      if(Chudau=='F')
      {
        Serial.println("aaaa");
        // Tách chuỗi
        int indexF = data.indexOf('F'); // tìm vị trí chữ F
        int indexA = data.indexOf('A', indexF + 1);
        int indexB = data.indexOf('B', indexA + 1);
        int indexC = data.indexOf('C', indexB + 1);


        ThetaX = data.substring(indexF + 1, indexA).toFloat();
        ThetaY = data.substring(indexA + 1, indexB).toFloat();
        ThetaZ = data.substring(indexB +1, indexC).toFloat();
              // ... các giá trị khác nếu cần

        Serial.println(ThetaX);
        Serial.println(ThetaY);
        Serial.println(ThetaZ);

        GoiGoc(ThetaX, ThetaY, ThetaZ);
      }
      else if(Chudau=='I')
      {
        Serial.println("bbbbb");
        int indexI = data.indexOf('I'); // tìm vị trí chữ F
        int indexA = data.indexOf('A', indexI + 1);
        int indexB = data.indexOf('B', indexA + 1);
        int indexC = data.indexOf('C', indexB + 1);

        ThetaX = data.substring(indexI + 1, indexA).toFloat();
        ThetaY = data.substring(indexA + 1, indexB).toFloat();
        ThetaZ = data.substring(indexB +1, indexC).toFloat();

        GoiGoc(ThetaX, ThetaY, ThetaZ);
      }      
      else if(Chudau=='S')
      {
          goHome_YZ();
          goHome_X();
          //ThetaX=ThetaY=ThetaZ=0;
          
      }   
      else if(Chudau=='H')
      {
        namcham_hut();
        delay(500);
      }      
      else if(Chudau=='T')
      {
        namcham_tha();
        delay(500);
      }
      else if(Chudau ='N')
      {
        int indexN = data.indexOf('N');
        int indexA = data.indexOf('A', indexN + 1);
        int indexB = data.indexOf('B', indexA + 1);
        int indexC = data.indexOf('C', indexB + 1);
        int indexD = data.indexOf('D', indexC + 1);
        int indexE = data.indexOf('E', indexD + 1);
        int indexF = data.indexOf('F', indexE + 1);
        int indexG = data.indexOf('G', indexF + 1);
        int indexH = data.indexOf('H', indexG + 1);
        int indexT = data.indexOf('T', indexH + 1);

        
        Serial.println("Processing Move Commands...");

        float A1_value = data.substring(indexN + 1, indexA).toFloat();
        float B1_value = data.substring(indexA + 1, indexB).toFloat();
        float C1_value = data.substring(indexB + 1, indexC).toFloat();

        float A2_value = data.substring(indexC + 1, indexD).toFloat();
        float B2_value = data.substring(indexD + 1, indexE).toFloat();
        float C2_value = data.substring(indexE + 1, indexF).toFloat();

        float A3_value = data.substring(indexF + 1, indexG).toFloat();
        float B3_value = data.substring(indexG + 1, indexH).toFloat();
        float C3_value = data.substring(indexH + 1, indexT).toFloat();

        GoiGoc(A1_value, 0, 0);     // ĐIỂM A
        GoiGoc(A1_value, B1_value, C1_value);     // ĐIỂM A
        delay(500);
        namcham_hut();
        delay(500);
        GoiGoc(A1_value, B1_value -40, C1_value);
        GoiGoc(105, 45, 36);     // ĐIỂM THẢ VẬT 
        delay(500);
        namcham_tha();
        GoiGoc(0, 0, 0);
        GoiGoc(A2_value,0, 0);     // ĐIỂM B
        GoiGoc(A2_value, B2_value, C2_value);     // ĐIỂM B
        delay(500);
        namcham_hut();
        delay(500);
        GoiGoc(A2_value, B2_value - 40, C2_value); 
        GoiGoc(82, 45, 40);     // ĐIỂM THẢ VẬT 
        delay(500);
        namcham_tha();
        GoiGoc(0, 0, 0);
        GoiGoc(A3_value,0, 0);     // ĐIỂM C
        GoiGoc(A3_value, B3_value, C3_value);     // ĐIỂM C
        delay(500);
        namcham_hut();
        delay(500);
        GoiGoc(A3_value, B3_value -40, C3_value);
        GoiGoc(60,40,35);     // ĐIỂM THẢ VẬT 
        delay(500);
        namcham_tha();
        GoiGoc(0, 0, 0);
      }         
      
      
      data = "";  // RESET LẠI GIÁ TRỊ DATA
    } 

}






int convertAngleToPulses_X(float angle) {
  return round(angle * pulsesPerDegree_X);
}
int convertAngleToPulses_YZ(float angle) {
  return round(angle * pulsesPerDegree_YZ);
}
// Hàm goHome để đưa hai động cơ về vị trí công tắc hành trình
void goHome_YZ() {
  bool stepperYRunning = true; // Trạng thái hoạt động của động cơ 1
  bool stepperZRunning = true; // Trạng thái hoạt động của động cơ 2

  // Lặp cho đến khi cả hai động cơ dừng
  while (stepperYRunning || stepperZRunning) {
    // Đọc trạng thái công tắc hành trình
    bool limitSwitchStateY = digitalRead(limitSwitchPinY);
    bool limitSwitchStateZ = digitalRead(limitSwitchPinZ);

    // Kiểm tra công tắc hành trình 1 cho stepper1
    if (limitSwitchStateY == LOW && stepperYRunning) {
      // Di chuyển stepperY  từng  10 bước
      stepperY.move(-100);
      stepperY.run();
    } else if (limitSwitchStateY == HIGH && stepperYRunning) {
      // Nếu stepperY chạm công tắc hành trình, dừng stepperY
      stepperY.stop();
      stepperYRunning = false;
      Serial.println("Stepper Y đã dừng do chạm công tắc hành trình.");
    }

    // Kiểm tra công tắc hành trình 2 cho stepper2
    if (limitSwitchStateZ == LOW && stepperZRunning) {
      // Di chuyển stepperZ từng bước
      stepperZ.move(-100);
      stepperZ.run();
    } else if (limitSwitchStateZ == HIGH && stepperZRunning) {
      // Nếu stepperZ chạm công tắc hành trình, dừng stepper2
      stepperZ.stop();
      stepperZRunning = false;
      Serial.println("Stepper Z đã dừng do chạm công tắc hành trình.");
    }
  }
  // Sau khi chạm công tắc hành trình, di chuyển stepper Y và Z thêm các bước tương ứng
  stepperY.move(1650);  // Di chuyển stepper Y thêm -1650 bước
  stepperZ.move(740);   // Di chuyển stepper Z thêm -500 bước

  // Tiến hành di chuyển động cơ cho đến khi hoàn thành
  while (stepperY.distanceToGo() != 0 || stepperZ.distanceToGo() != 0) {
    stepperY.run();
   // delay(200);
    stepperZ.run();
  }

  // Sau khi hoàn thành, đặt lại vị trí hiện tại
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);

  // Dừng hoàn toàn hai động cơ sau khi cả hai đã về home
  //digitalWrite(enablePin, HIGH); // Tắt nguồn động cơ (HIGH = OFF)
  Serial.println("STEP Y VÀ STEP Z DỪNG");
}
void goHome_X() {
  bool stepperXRunning = true; // Trạng thái hoạt động của động cơ 

  // Lặp cho đến khi động cơ 3 dừng
  while (stepperXRunning) {
    // Đọc trạng thái công tắc hành trình
    bool limitSwitchStateX = digitalRead(limitSwitchPinX);

    // Kiểm tra công tắc hành trình 3 cho stepper3
    if (limitSwitchStateX == LOW && stepperXRunning) {
      // Di chuyển stepper3 từng bước
      stepperX.move(100);
      stepperX.run();
    } else if (limitSwitchStateX == HIGH && stepperXRunning) {
      // Nếu stepper3 chạm công tắc hành trình, dừng stepper3
      stepperX.stop();
      stepperXRunning = false;
      Serial.println("Stepper 3 đã dừng do chạm công tắc hành trình 3.");
    }
  }
  stepperX.setCurrentPosition(0);
}

void run(int X, int Y, int Z)
{
  Serial.println(X);
  Serial.println(Y);
  Serial.println(Z);
  stepperX.moveTo(X);
  stepperY.moveTo(Y);
  stepperZ.moveTo(Z+Y);
  while(stepperX.distanceToGo() != 0 or stepperY.distanceToGo() != 0 or stepperZ.distanceToGo() != 0)
  {
    stepperX.run();
    stepperY.run();
    stepperZ.run();
  }  
  Serial.println("Stepper X, Y, Z đã hoàn thành di chuyển.");
}

void GoiGoc(float angleX,float angleY,float angleZ)
{
        int xPulse = convertAngleToPulses_X(angleX);
        int yPulse = convertAngleToPulses_YZ(angleY);
        int zPulse = convertAngleToPulses_YZ(angleZ);
        // Chạy động cơ với số xung đã chuyển đổi
        run(xPulse, yPulse, zPulse);
}

void namcham_hut()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
}
void namcham_tha()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
}