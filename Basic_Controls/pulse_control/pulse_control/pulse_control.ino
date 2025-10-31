const int pulse_pin = 20;
const int trigger_pin = 16;
const int mode_pin = 0;

void setup()
{
  pinMode(pulse_pin, OUTPUT);
  pinMode(mode_pin, INPUT);
  // digitalWrite(pulse_pin, HIGH);
  digitalWriteFast(pulse_pin, LOW);
  CORE_PIN20_PADCONFIG |= 0xF9;

  pinMode(trigger_pin, OUTPUT);
  digitalWriteFast(trigger_pin, LOW);
  CORE_PIN16_PADCONFIG |= 0xF9;

}

void continuous()
{
  digitalWriteFast(pulse_pin, HIGH);
}

void pulse()
{
  digitalWriteFast(pulse_pin, HIGH);
  digitalWriteFast(pulse_pin, LOW);
  delayNanoseconds(200);
  // delayNanoseconds(220);
  digitalWriteFast(trigger_pin, HIGH);
  delayNanoseconds(10);
  digitalWriteFast(trigger_pin, LOW);
  delayNanoseconds(733);
  // delayNanoseconds(713);
}

void loop()
{
  if (digitalReadFast(mode_pin) == HIGH) {
    continuous();
  } else {
    pulse();
  }
}