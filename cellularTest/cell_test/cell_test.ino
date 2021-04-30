
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    //wait for serial port
  }

  Serial1.begin(9600);
}

void loop() {
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
}

// resource on the cell module: https://lastminuteengineers.com/sim800l-gsm-module-arduino-tutorial/

// AT
// AT+CFUN?
// AT+SAPBR=3,1,"Contype","GBRS"
// AT+CSTT="3gprs","3gprs","3gprs" ... this will differ by cell provider, user, password
// AT+SAPBR=1,1
// AT+HTTPINIT
// AT+HTTPPARA="CID",1
// AT+HTTPPARA="URL","http://example.com"
// AT+HTTPACTION=0
// AT+HTTPREAD
// AT+HTTPTERM
// reapeat from httpinit
