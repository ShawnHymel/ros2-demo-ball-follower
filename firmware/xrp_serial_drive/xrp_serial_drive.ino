/**
 * XRP Serial Drive
 * 
 * Send strings over serial (115200 baud rate) in CSV format:
 *
 *  "<left speed>, <right speed>, <left dir>, <right dir>\n"
 *
 * Speed is 0..100. dir is 0 or 1
 * 
 * License: 0BSD https://opensource.org/license/0bsd
 */

// Settings
#define DEBUG 0
const int min_speed = 120;
constexpr int max_chars = 64;
constexpr int max_vals = 4;

// Pins
const int mt_l_dir_pin = 6;
const int mt_l_pwm_pin = 7;
const int mt_r_dir_pin = 14;
const int mt_r_pwm_pin = 15;

// Buffers
char input_buf[max_chars];
int vals[max_vals];
int buf_idx = 0;

void parse_csv_ints(char* input) {
  byte val_count = 0;

  // Break string apart based on delimiter
  char* token = strtok(input, ", ");

  // Save values and continue breaking apart string
  while (token != NULL && val_count < max_vals) {
    vals[val_count++] = atoi(token);
    token = strtok(NULL, ", ");
  }
}

void setup() {
  
  // Configure serial
  Serial.begin(115200);

  // Configure drive pins
  pinMode(mt_l_dir_pin, OUTPUT);
  pinMode(mt_l_pwm_pin, OUTPUT);
  pinMode(mt_r_dir_pin, OUTPUT);
  pinMode(mt_r_pwm_pin, OUTPUT);

  // Don't move
  analogWrite(mt_l_pwm_pin, 0);
  analogWrite(mt_r_pwm_pin, 0);

  // Initialize values array
  memset(vals, 0, sizeof(vals));
}

void loop() {

  // Receive bytes from serial
  while (Serial.available() > 0) {

    // Read and parse the line-terminated string
    char received = Serial.read();
    if (received == '\n') {
      input_buf[buf_idx] = '\0';
      parse_csv_ints(input_buf);
      buf_idx = 0;
    }
    else if (received != '\r' && buf_idx < max_chars - 1) {
      input_buf[buf_idx++] = received;
    }
  }

  // Debug received message
#if DEBUG
  Serial.print("Parsed values: ");
  for (int i = 0; i < max_vals; i++) {
    Serial.print(vals[i]);
    if (i < max_vals - 1) Serial.print(", ");
  }
  Serial.println();
#endif

  // Set motor directions
  int l_dir = vals[2] > 0 ? 1 : 0;
  int r_dir = vals[3] > 0 ? 1 : 0;
  digitalWrite(mt_l_dir_pin, l_dir);
  digitalWrite(mt_r_dir_pin, r_dir);

  // Calculate motor speed: map 1..100 to min_speed..255
  int l_speed = vals[0] > 0 ? vals[0] : 0;
  int r_speed = vals[1] > 0 ? vals[1] : 0;
  if (l_speed > 0) {
    l_speed = map(l_speed, 1, 100, min_speed, 255);
  }
  if (r_speed > 0) {
    r_speed = map(r_speed, 1, 100, min_speed, 255);
  }
  
  // Set motor speeds
  analogWrite(mt_l_pwm_pin, l_speed);
  analogWrite(mt_r_pwm_pin, r_speed);
}
