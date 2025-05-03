#ifdef BATTERY
    void battery_level(){
        float value = analogRead(BATTERY_PIN);
        float pin_volt = value / MAX_ARD_VAL * MAX_PIN_VAL;
        float true_bat_volt = pin_volt * (R7 + R8) / R8;
        Serial.println("Battery Voltage:");
        Serial.println(true_bat_volt);
    }
#endif