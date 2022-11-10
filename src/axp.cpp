// Basic config
#include "Arduino.h"
#include "axp20x.h"

AXP20X_Class axp;

void AXP192_power(bool state) 
{
    if(state) 
    {
        axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);  // Lora on T-Beam V1.0
        axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // Gps on T-Beam V1.0
        axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // OLED on T-Beam v1.0
        axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
        axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
        // axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
        axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
    } 
    else 
    {
        axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);  // Lora on T-Beam V1.0
        axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // Gps on T-Beam V1.0
        axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF); // OLED on T-Beam v1.0
        axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
        axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
        axp.setChgLEDMode(AXP20X_LED_OFF);
    }
}

void AXP192_init() 
{      
    if (axp.begin(Wire, AXP192_SLAVE_ADDRESS)) 
    {
        Serial.println("AXP192 axp initialization failed");
    }
    else 
    {
        Serial.println("Initialzation...");
        // configure AXP192
        axp.setTimeOutShutdown(false);          // no automatic shutdown
        axp.setTSmode(AXP_TS_PIN_MODE_DISABLE); // TS pin mode off to save power

        // switch ADCs on
        axp.adc1Enable(AXP202_BATT_VOL_ADC1, true);
        axp.adc1Enable(AXP202_BATT_CUR_ADC1, true);
        axp.adc1Enable(AXP202_VBUS_VOL_ADC1, true);
        axp.adc1Enable(AXP202_VBUS_CUR_ADC1, true);

        // switch power rails on
        AXP192_power(true);

        Serial.println("AXP192 axp initialized");
    }
}
