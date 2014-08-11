#define   LED_PIN  (3)
void setup(void)
{
    pinMode(LED_PIN, OUTPUT);
    RasPiDeckGPIO5v(LED_PIN);
}

void loop(void)
{
    digitalWrite(LED_PIN, 1);
    delay(300);
    digitalWrite(LED_PIN, 0);
    delay(300);
}
