# FUSB302_ESP32
Example with ESP32 and FUSB302

This example uses the library (V4.1.1) provided by ON Semi.

The main loop verifies if there is any interrupt from GPIO pin or timer and process the USB Type C state.

When an event happens (connect, disconnect), the main `if` from `while (1)` will be called multiple times. There will be a console message when the Type C protocol finished the communication.
