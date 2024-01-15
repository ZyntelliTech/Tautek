Blinky
######

Overview
********

| This sample is to blink the RGB LEDs. These are the GPIO pins to control the RGB LED.
| #define RED_LED_NODE DT_ALIAS(redled)
| #define BLUE_LED_NODE DT_ALIAS(blueled)
| #define GREEN_LED_NODE DT_ALIAS(greenled)
| 
| static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(RED_LED_NODE, gpios);
| static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(BLUE_LED_NODE, gpios);
| static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(GREEN_LED_NODE, gpios);
| 
| RED   ----- P0.13
| BLUE  ----- P0.14
| GREEN ----- P0.15

