#include <Readline.h>

ReadLine r;

void setup() {
  Serial.begin(57600);
}

char *line;
void loop() {
  if ((line = r.feed(&Serial)) != NULL) {
    Serial.print("got line [");
    Serial.print(line);
    Serial.println("]");
  }
}
