#pragma once

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

// LCD object: address 0x27, 16 columns, 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Store displayed colors as 3-letter abbreviations
// indices 0..3 correspond to antennas 1..4
char lcdColors[4][4] = {
  "RED",   // Antenna 1
  "GRE",   // Antenna 2
  "GRE",   // Antenna 3
  "PUR"    // Antenna 4
};

unsigned long lcdPreviousTime = 0;
const unsigned long lcdInterval = 2000;   // 2 seconds
int lcdAntennaIndex = 0;

// ----------------------------------------------------
// Convert one-letter code to display text
// R -> RED
// G -> GRE
// B -> BLU
// P -> PUR
// blank -> ???
// ----------------------------------------------------
void setOneColorFromLetter(int index, char letter)
{
  if (index < 0 || index > 3) return;

  if (letter == 'R' || letter == 'r') {
    strcpy(lcdColors[index], "RED");
  }
  else if (letter == 'G' || letter == 'g') {
    strcpy(lcdColors[index], "GRE");
  }
  else if (letter == 'B' || letter == 'b') {
    strcpy(lcdColors[index], "BLU");
  }
  else if (letter == 'P' || letter == 'p') {
    strcpy(lcdColors[index], "PUR");
  }
  else {
    strcpy(lcdColors[index], "???");
  }
}

// ----------------------------------------------------
// Draw the current antenna on the LCD
// ----------------------------------------------------
void lcdShowAntenna(int index)
{
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Robot Ready");

  lcd.setCursor(0, 1);
  lcd.print("Ant ");
  lcd.print(index + 1);
  lcd.print(" ");
  lcd.print(lcdColors[index]);
}

// ----------------------------------------------------
// Parse commands like:
// D R,G,P,B
// D ,R,,P
//
// After "D " there should be 4 comma-separated fields.
// Each field may be:
//   R, G, B, P, or blank
// Blank becomes ???
// ----------------------------------------------------
void lcdUpdateFromCommand(char *cmd)
{
  // Expect cmd to point to the text AFTER "D "
  // Example:
  //   "R,G,P,B"
  //   ",R,,P"

  int antIndex = 0;
  int tokenLen = 0;
  char token[5] = {0};

  for (int i = 0; ; i++) {
    char c = cmd[i];

    if (c == ',' || c == '\n' || c == '\r' || c == '\0') {
      // finish current token
      if (antIndex < 4) {
        if (tokenLen == 0) {
          strcpy(lcdColors[antIndex], "???");
        }
        else {
          setOneColorFromLetter(antIndex, token[0]);
        }
      }

      antIndex++;
      tokenLen = 0;
      memset(token, 0, sizeof(token));

      if (c == '\n' || c == '\r' || c == '\0' || antIndex >= 4) {
        break;
      }
    }
    else {
      if (tokenLen < 4) {
        token[tokenLen] = c;
        tokenLen++;
      }
    }
  }

  // Any missing remaining antennas become ???
  while (antIndex < 4) {
    strcpy(lcdColors[antIndex], "???");
    antIndex++;
  }

  lcdAntennaIndex = 0;
  lcdShowAntenna(lcdAntennaIndex);
}

// ----------------------------------------------------
// Init function to call from main setup()
// ----------------------------------------------------
void lcdDisplayInit(void)
{
  lcd.init();
  delay(50);
  lcd.backlight();
  lcd.clear();

  lcdShowAntenna(lcdAntennaIndex);
  lcdPreviousTime = millis();
}

// ----------------------------------------------------
// Task function to call repeatedly from loop()
// ----------------------------------------------------
void lcdDisplayTask(void)
{
  unsigned long lcdCurrentTime = millis();

  if (lcdCurrentTime - lcdPreviousTime >= lcdInterval) {
    lcdPreviousTime = lcdCurrentTime;

    lcdAntennaIndex++;
    if (lcdAntennaIndex >= 4) {
      lcdAntennaIndex = 0;
    }

    lcdShowAntenna(lcdAntennaIndex);
  }
}