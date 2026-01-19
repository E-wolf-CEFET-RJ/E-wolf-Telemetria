#include <RtcDS1302.h>

// Pinos
#define PIN_CLK   6
#define PIN_DATA  7
#define PIN_RST   8

ThreeWire myWire(PIN_DATA, PIN_CLK, PIN_RST);
RtcDS1302<ThreeWire> rtc(myWire);

// Converte __DATE__ e __TIME__ para RtcDateTime
RtcDateTime compileDateTime() {
  const char* date = __DATE__; // "Jan 17 2026"
  const char* time = __TIME__; // "14:32:45"

  char monthStr[4];
  int day, year;
  sscanf(date, "%3s %d %d", monthStr, &day, &year);

  int hour, minute, second;
  sscanf(time, "%d:%d:%d", &hour, &minute, &second);

  int month;
  if (!strcmp(monthStr, "Jan")) month = 1;
  else if (!strcmp(monthStr, "Feb")) month = 2;
  else if (!strcmp(monthStr, "Mar")) month = 3;
  else if (!strcmp(monthStr, "Apr")) month = 4;
  else if (!strcmp(monthStr, "May")) month = 5;
  else if (!strcmp(monthStr, "Jun")) month = 6;
  else if (!strcmp(monthStr, "Jul")) month = 7;
  else if (!strcmp(monthStr, "Aug")) month = 8;
  else if (!strcmp(monthStr, "Sep")) month = 9;
  else if (!strcmp(monthStr, "Oct")) month = 10;
  else if (!strcmp(monthStr, "Nov")) month = 11;
  else if (!strcmp(monthStr, "Dec")) month = 12;

  return RtcDateTime(year, month, day, hour, minute, second);
}

void setup() {
  Serial.begin(9600);
  rtc.Begin();

  if (!rtc.GetIsRunning()) {
    rtc.SetIsRunning(true);
  }

  RtcDateTime rtcNow = rtc.GetDateTime();
  RtcDateTime buildTime = compileDateTime();

  // diferença em segundos
  int32_t diff =
    abs((int32_t)rtcNow.TotalSeconds() -
        (int32_t)buildTime.TotalSeconds());

  // Se RTC inválido OU diferença grande → sincroniza
  if (!rtc.IsDateTimeValid() || diff > 120) { // 2 minutos
    Serial.println("Sincronizando RTC com horario de compilacao...");
    rtc.SetDateTime(buildTime);
  } else {
    Serial.println("RTC OK, mantendo horario.");
  }
}

void loop() {
  RtcDateTime now = rtc.GetDateTime();

  Serial.print(now.Day());
  Serial.print("/");
  Serial.print(now.Month());
  Serial.print("/");
  Serial.print(now.Year());
  Serial.print(" ");

  Serial.print(now.Hour());
  Serial.print(":");
  Serial.print(now.Minute());
  Serial.print(":");
  Serial.println(now.Second());

  delay(1000);
}
