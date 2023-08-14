#include <MsTimer2.h>

volatile int send_data_flag = 0;

void toggleFlag();

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
  MsTimer2::set(5000, toggleFlag);
  MsTimer2::start();
}

void loop() {  
  char send_data1[7]; // Increased the size by 1 to accommodate the delimiter character
  char send_data2[7]; // Increased the size by 1 to accommodate the delimiter character
  char send_data3[7]; // Increased the size by 1 to accommodate the delimiter character
  char send_data4[7]; // Increased the size by 1 to accommodate the delimiter character
  char send_data5[7]; // Increased the size by 1 to accommodate the delimiter character
  char delimiter = '<';
  
  String data1[2];
  data1[0]="S1"; //
  data1[1]="R1"; //

  String data2[2];
  data2[0]="S2"; //
  data2[1]="R2"; //

  String data3[2];
  data3[0]="S3"; //
  data3[1]="R3"; //

  String data4[2];
  data4[0]="S4"; //
  data4[1]="R4"; //

  String data5[2];
  data5[0]="S5"; //
  data5[1]="R5"; //

  sprintf(send_data1, "%s,%s%c", data1[0].c_str(), data1[1].c_str(), delimiter);
  sprintf(send_data2, "%s,%s%c", data2[0].c_str(), data2[1].c_str(), delimiter);
  sprintf(send_data3, "%s,%s%c", data3[0].c_str(), data3[1].c_str(), delimiter);
  sprintf(send_data4, "%s,%s%c", data4[0].c_str(), data4[1].c_str(), delimiter);
  sprintf(send_data5, "%s,%s%c", data5[0].c_str(), data5[1].c_str(), delimiter);

  if (Serial2.available() > 0) {
    String cmdString = "at+qd1?";
    String recString = Serial2.readStringUntil('\n');
    recString.trim(); // Remove leading/trailing whitespace

//    Serial.print("Received String: [");
//    Serial.print(recString);
//    Serial.println("]");

    if (recString.equals(cmdString)) {
      if (send_data_flag == 0) {
        Serial.print("Send Data: ");
        Serial.println(send_data1);
        Serial2.print(send_data1); 
      }
      else if(send_data_flag == 1) {
        Serial.print("Send Data2: ");
        Serial.println(send_data2);
        Serial2.print(send_data2);
      }
      else if(send_data_flag == 2) {
        Serial.print("Send Data3: ");
        Serial.println(send_data3);
        Serial2.print(send_data3);
      }
      else if(send_data_flag == 3) {
        Serial.print("Send Data4: ");
        Serial.println(send_data4);
        Serial2.print(send_data4);
      }
      else if(send_data_flag == 4) {
        Serial.print("Send Data5: ");
        Serial.println(send_data5);
        Serial2.print(send_data5);
      }
    }
  }
}

void toggleFlag() {
  send_data_flag++;
  if (send_data_flag > 4) send_data_flag = 0;
}
