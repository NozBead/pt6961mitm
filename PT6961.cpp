#define CLK PIN2
#define STB PIN3
#define DOUT PIN4
#define DIN PIN5
#define ROTARY PIN6

enum state {COMMAND, RESPONSE, NONE};

volatile enum state current_state;

#define KEY_REPEAT 6

#define KEY_MAP_SIZE 3
unsigned char key_map[KEY_MAP_SIZE];

void reset_command_transfer() { 
  current_state = NONE;
  DDRD |= (1 << DOUT);
  EIMSK = (1 << INT0) | (1 << INT1);
}

void deafen_controller() {
  DDRD |= (1 << STB);
  PORTD |= (1 << STB);
  DDRD &= ~(1 << STB);
}

void wait_command() {
  EIMSK = 0x0;
  DDRD &= ~(1 << DOUT);
}

unsigned char read_bit(unsigned char mask) {
  return PIND & (1 << DIN) ? mask : 0;
}

void write_bit(unsigned char *value, unsigned char mask) {
  if (*(value) & mask)  {
    PORTD = (1 << DOUT);
  }
  else {
    PORTD &= ~(1 << DOUT);
  }
}

void clock_command() {
  static unsigned char mask = 0x1;
  static unsigned char command = 0;
  delayMicroseconds(2);
  command |= read_bit(mask);
  mask <<= 1;

  if (mask == 0x80) {
    if (command == 0x42) {
      current_state = RESPONSE;
      deafen_controller();
      mask = 0x1;
      command = 0;
      EIMSK = (1 << INT0);
      EICRA = (1 << ISC11) | (0 << ISC10) | (1 << ISC01) | (1 << ISC00);
    }
    else {
      mask = 0x1;
      command = 0;
      reset_command_transfer();
    }
  }
}

void clock_response() {
  static unsigned char mask = 0x1;
  static int repeat_count = 0;
  static int key_index = 0;

  write_bit(key_map+key_index, mask);
  mask <<= 1;

  if (mask == 0) {
    mask = 0x1;
    key_index++;
    if (key_index == KEY_MAP_SIZE) {
      key_index = 0;
      repeat_count++;
      if (repeat_count == KEY_REPEAT) {
        repeat_count = 0;
        wait_command();
      }
      else {
        mask = 0x1;
        reset_command_transfer();
      }
    }
  }
}

void handle_clock() {
  if (current_state == COMMAND) {
    clock_command();
  }
  else if (current_state == RESPONSE) {
    clock_response();
  }
}

void mark_command() {
  current_state = COMMAND;
}

void setup() {
  Serial.begin(9600);
  DDRD = (0 << CLK) | (0 << STB) | (1 << DOUT) | (0 << DIN) | (0 << ROTARY);
  attachInterrupt(digitalPinToInterrupt(CLK), handle_clock, FALLING);
  attachInterrupt(digitalPinToInterrupt(STB), mark_command, FALLING);
  wait_command();
}

void loop() {
  int read = Serial.readBytes(key_map, 3);
  if (read == 3) {
    reset_command_transfer();
  }
  else {
    unsigned char rotary_state = PIND & (1 << ROTARY);
    unsigned char rotating = 0;
    for (int i = 0 ; i < 100 && !rotating ; i++) {
      delay(10);
      unsigned char new_state = PIND & (1 << ROTARY);
      if (rotary_state != new_state) {
        rotating = 1;
      }
    }

    Serial.write(rotating);
    Serial.flush();
  }
}
