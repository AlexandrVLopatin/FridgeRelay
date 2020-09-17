#include <Arduino.h>

#include <avr/eeprom.h>
#include <util/delay.h>

#include "tm1637.h"
#include "tm1637.c"

#include <OneWire.h>
#include <DallasTemperature.h>

#define DS18B20_PIN PB2

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress thermometer;

#define RELAY_TYPE_LOW_TRIGGER 0
#define RELAY_TYPE_HIGH_TRIGGER 1

// Settings =================================================

#define INDUCTIVE_CURRENT_CALMING 500ul  // milliseconds
#define RELAY_DELAY 1000ul               // milliseconds
#define BUTTON_HOLDING_TIME 5000ul       // milliseconds
#define EEPROM_SCHEDULER_INTERVAL 1000ul // milliseconds
#define TEMPERATURE_READ_INTERVAL 1000ul // milliseconds

#define RELAY_TYPE RELAY_TYPE_HIGH_TRIGGER

#define PIN_RELAY PB0
#define PIN_TRIAC PB1

#ifdef __AVR_ATtinyX5__
#define PIN_BTN PB2
#else
#define PIN_BTN 2
#endif

// ==========================================================

#define _A 0x77
#define _E 0x79
#define _F 0x71
#define _N 0x37
#define _O 0x3f
#define _P 0x73
#define _S 0x6d
#define _U 0x3e
#define _n 0x54
#define _o 0x5C
#define _t 0x78
#define _degree 0x63
#define _top 0x1
#define _bottom 0x8
#define _empty 0x00

unsigned long switchTime;
unsigned long timerDisplay;
unsigned long timerBtn;
unsigned long timerSetupSwitch;
unsigned long timerEEPROM;
unsigned long timerTemperature;

unsigned long timerOnTime = 15ul * 60000ul;
unsigned long timerOffTime = 15ul * 60000ul;

#define INPUT_MODE_TEMPERATURE 0
#define INPUT_MODE_BUTTON 1

uint8_t inputMode = INPUT_MODE_TEMPERATURE;

uint8_t eepromUpdateFlag = 0;
uint8_t mode = 1;
uint8_t setupMode = 0;

volatile uint8_t btnPressed = 0;

uint8_t onTime = 30;
uint8_t offTime = 30;

void displayRunningLine(const uint8_t data[])
{
    uint8_t segments[4 + 8];
    for (uint8_t i = 0; i < 4; i++)
        segments[i] = 0x00;
    for (uint8_t i = 0; i < 4; i++)
        segments[i + 4] = data[i];
    for (uint8_t i = 4 + 4; i < 4 + 8; i++)
        segments[i] = 0x00;
    for (uint8_t i = 0; i < 4 + 4; i++)
    {
        for (uint8_t j = 0; j <= 3; j++)
            TM1637_display_segments(j, segments[i + j + 1]);
        _delay_ms(200);
    }
}

void ON()
{
    PORTB |= (1 << PIN_TRIAC);

    _delay_ms(INDUCTIVE_CURRENT_CALMING);

#if RELAY_TYPE == RELAY_TYPE_HIGH_TRIGGER
    PORTB |= (1 << PIN_RELAY);
#else
    PORTB &= ~(1 << PIN_RELAY);
#endif

    _delay_ms(RELAY_DELAY);

    PORTB &= ~(1 << PIN_TRIAC);

    mode = 1;
    switchTime = millis();
    static uint8_t data[] = {_empty, _empty, _O, _n};
    displayRunningLine(data);
}

void OFF()
{
    PORTB |= (1 << PIN_TRIAC);

    _delay_ms(RELAY_DELAY);

#if RELAY_TYPE == RELAY_TYPE_HIGH_TRIGGER
    PORTB &= ~(1 << PIN_RELAY);
#else
    PORTB |= (1 << PIN_RELAY);
#endif

    _delay_ms(INDUCTIVE_CURRENT_CALMING);

    PORTB &= ~(1 << PIN_TRIAC);

    mode = 0;
    switchTime = millis();
    static uint8_t data[] = {_empty, _O, _F, _F};
    displayRunningLine(data);
}

void timerRoutine()
{
    if (setupMode > 0)
        return;

    const unsigned long diff = millis() - switchTime;

    if (mode == 1 && (diff > timerOnTime))
        OFF();
    else if (mode == 0 && (diff > timerOffTime))
        ON();
}

void scheduleEEPROMUpdate()
{
    eepromUpdateFlag = true;
    timerEEPROM = millis();
}

void eepromRoutine()
{
    if (eepromUpdateFlag && millis() - timerEEPROM > EEPROM_SCHEDULER_INTERVAL)
    {
        eepromUpdateFlag = false;
        eeprom_write_byte((uint8_t *)0, onTime);
        eeprom_write_byte((uint8_t *)1, offTime);
    }
}

void displayTime(const unsigned long time)
{
    TM1637_display_digit(2, time / 10);
    TM1637_display_digit(3, time % 10);
}

void displayRoutine()
{
    const unsigned long m = millis();

    if (millis() - timerDisplay < 100)
        return;

    timerDisplay = m;

    if (setupMode == 0)
    {
        unsigned long t;

        if (mode == 1)
        {
            TM1637_display_segments(0, _degree);
            t = switchTime + timerOnTime - m;
        }
        else if (mode == 0)
        {
            TM1637_display_segments(0, _o);
            t = switchTime + timerOffTime - m;
        } else {
            t = 0;
        }

        t = t / 60000 + 1;

        displayTime(t);
    }
    else
    {
        TM1637_display_digit(0x01, B01011101);
        if (setupMode == 1)
        {
            TM1637_display_segments(0, _top);
            displayTime(onTime);
        }
        else if (setupMode == 2)
        {
            TM1637_display_segments(0, _bottom);
            displayTime(offTime);
        }

        if (m - timerSetupSwitch > 3000)
        {
            timerSetupSwitch = m;
            setupMode++;
            if (setupMode == 3)
            {
                setupMode = 0;
                timerOnTime = onTime * 60000ul;
                timerOffTime = offTime * 60000ul;
                static uint8_t data[] = {_S, _A, _U, _E};
                displayRunningLine(data);
                scheduleEEPROMUpdate();
            }

            TM1637_clear();
        }
    }
}

void btnRoutine()
{
    if (btnPressed == 1)
    {
        if (setupMode == 0)
        {
            //const uint8_t btnPinState = PINB & (1 << PIN_BTN); //digitalRead(PIN_BTN);
            const uint8_t btnPinState = digitalRead(PIN_BTN);
            if (btnPinState == HIGH && (millis() - timerBtn > BUTTON_HOLDING_TIME))
            {
                setupMode = 1;

                static uint8_t data[] = {_empty, _S, _E, _t};
                displayRunningLine(data);

                timerSetupSwitch = millis();
                btnPressed = 0;
            }
            else if (btnPinState == LOW)
            {
                btnPressed = 0;
            }
            return;
        }
        else if (setupMode == 1)
        {
            onTime += 1;
            if (onTime > 60)
                onTime = 1;
            timerSetupSwitch = millis();
            btnPressed = 0;
        }
        else if (setupMode == 2)
        {
            offTime += 1;
            if (offTime > 60)
                offTime = 1;
            timerSetupSwitch = millis();
            btnPressed = 0;
        }
    }
}

[[noreturn]] void temperatureRoutine()
{
    if (millis() - timerTemperature > TEMPERATURE_READ_INTERVAL)
    {
        sensors.requestTemperatures();

        timerTemperature = millis();

        if (sensors.hasAlarm(thermometer))
        {
            OFF();
            uint8_t data[] = {_N, _E, _P, _E};
            for (uint8_t i = 0; i < 4; i++)
                TM1637_display_segments(i, data[i]);

            while (1)
            {
                _delay_ms(1000);
            }
        }
    }
}

void setup()
{
    DDRB |= (1 << PIN_RELAY) | (1 << PIN_TRIAC);

    _delay_ms(50);

    sensors.begin();
    if (sensors.getAddress(thermometer, 0))
    {
        sensors.setHighAlarmTemp(thermometer, 60);
        sensors.setLowAlarmTemp(thermometer, -20);
        inputMode = INPUT_MODE_TEMPERATURE;
    }
    else
    {
        inputMode = INPUT_MODE_BUTTON;
        DDRB &= ~(1 << PIN_BTN);

        cli();
#ifdef __AVR_ATtinyX5__
        MCUCR |= (1 << ISC00) | (1 << ISC01);
        GIMSK |= (1 << INT0);
#else
        EICRA |= (1 << ISC00) | (1 << ISC01);
        EIMSK |= (1 << INT0);
#endif
        sei();
    }

    onTime = eeprom_read_byte((uint8_t *)0);
    if (onTime > 60 || onTime == 0)
        onTime = 15;

    offTime = eeprom_read_byte((uint8_t *)1);
    if (offTime > 60 || offTime == 0)
        offTime = 15;

    timerOnTime = onTime * 60000ul;
    timerOffTime = offTime * 60000ul;

    switchTime = timerDisplay = timerTemperature = millis();

    TM1637_init(1 /*enable*/, 5 /*brightness*/);
    TM1637_clear();

    ON();
}

ISR(INT0_vect)
{
    btnPressed = 1;
    timerBtn = millis();
}

void loop()
{
    timerRoutine();
    displayRoutine();

    if (inputMode == INPUT_MODE_BUTTON)
    {
        btnRoutine();
        eepromRoutine();
    }
    else if (inputMode == INPUT_MODE_TEMPERATURE)
    {
        temperatureRoutine();
    }
}
