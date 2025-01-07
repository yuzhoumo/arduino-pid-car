#include <EEPROM.h>

size_t log_index = 0;

void check_serial_command() {
  if (!Serial.available()) {
    return;
  }
  switch (Serial.read()) {
    case 'w': write_log("this is a test! "); break;
    case 'p': print_log(); break;
    case 'c': clear_log(); break;
  }
}

void clear_log() {
  log_index = 0;
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
}

void end_log() {
  write_log("encoder ticks: ");
  write_log_ulong(encoder_ticks);
  write_log("\n");

  write_log("total time (ms): ");
  write_log_ulong(millis() - start_time);
  write_log("\n");
}

void write_log(String log) {
  if (log.length() > (log_index + EEPROM.length() - 1)) {
    return;
  }

  for (int i = 0; i < log.length(); i++) {
    EEPROM.write(log_index, log[i]);
    log_index++;
  }

  EEPROM.write(log_index, 0);
}

void write_log_ulong(unsigned long n) {
  char buf[10];
  write_log(String(itoa(n, buf, 10)));
}

void print_log() {
  String log = "";
  for (int i = 0; i < EEPROM.length(); i++) {
    char c = EEPROM.read(i);
    if (c == 0) break; // stop at null terminator
    log += c;
  }
  Serial.println(log);
}
