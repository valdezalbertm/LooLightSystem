#include <Arduino.h>

/**
 * ATTiny85
 * SRAM: 512 Bytes
 * Flash Memory: 8kb
 */

const int DOOR_PIN = 12; // magnetic reed sensor // digital
// const int pin_ds2 = A1; // middle // pwm
// const int pin_ds1 = A0; // bowl // pwm
const int PIN_DS2 = 10; // TODO: change this port, middle // pwm
const int PIN_DS1 = 11; // TODO: change this port, bowl // pwm
const int PIN_PIR = 8; // digital
const int RELAY_PIN = 7; //digital

/*** DOOR PIN ***/
bool is_door_open = false; // When the button is pressed, it means the door is opened, it's value will become true
// unsigned long door_last_open_at;
// This is the interval that the door is open for a long time
const unsigned int OPEN_DOOR_THRESHOLD_INTERVAL_MS = 10000;
unsigned long open_door_threshold_ms;
bool is_door_stale;
bool is_door_just_closed;

/*** DISTANCE SENSOR 2 - MIDDLE ***/
int pin_ds2_val; // analog
const unsigned int DS2_THRESHOLD_INTERVAL_MS = 10000;

/*** DISTANCE SENSOR 1 - BOWL ***/
int pin_ds1_val; // analog
const unsigned int DS1_THRESHOLD_INTERVAL_MS = 20000;

/*** PIR SENSOR ***/
bool has_pir_detected; // digital
const unsigned int PIR_THRESHOLD_INTERVAL_MS = 5000;
unsigned int pir_last_activity_ms;

bool is_led_light; // digital

bool has_activity = false;

// time to remain the LED/relay/bulb until the millis reached this time
// specific sensors has each threshold
unsigned long remain_active_until_ms;

bool hasPreviousActivity();
void setRemainActiveUntil(int remain_active_threshold_ms, bool force);
void fetchActivity();

unsigned long must_have_activity_until_ms;
unsigned int MUST_HAVE_ACTIVITY_THRESHOLD_MS = 5000;
// const unsigned int SOMEONE_INSIDE_ACTIVE_MS = 65535; // this is 6 minutes
const unsigned int SOMEONE_INSIDE_ACTIVE_MS = 7333; // this is 6 minutes
bool is_someone_inside;

// cppcheck-suppress unusedFunction
void setup()
{
    Serial.begin(9600);

    is_door_stale = false;
    remain_active_until_ms = millis() + 5000;
    must_have_activity_until_ms = 0;
    is_door_just_closed = false;
    is_someone_inside = false;
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(PIN_PIR, INPUT);
    pinMode(PIN_DS1, INPUT);
    pinMode(PIN_DS2, INPUT);
}

// cppcheck-suppress unusedFunction
void loop()
{
    fetchActivity();
    Serial.print("current milli:");
    Serial.print(millis());
    Serial.print(" remain_active_until_ms:");
    Serial.println(remain_active_until_ms);

    is_door_open = digitalRead(DOOR_PIN);
    if (is_door_open) {
        // Button is pushed
        Serial.println("Door is now/still open");
        is_door_just_closed = false;
        is_someone_inside = false;

        unsigned long current_millis = millis();

        if (is_door_stale) {
            if (hasPreviousActivity()) {
                Serial.println("Someone got inside after a long time!");
                // there's a sudden activity, someone got inside after a long time
                is_door_stale = false;
                digitalWrite(RELAY_PIN, true);
            } else {
                Serial.println("Door remains stale.");
                // do nothing, door is stale, and no activity
                // let's keep relay closed
            }
        } else if (open_door_threshold_ms > 0) {
            if (hasPreviousActivity()) {
                Serial.println("Door is open, and someone inside, maybe washing feet");
                digitalWrite(RELAY_PIN, true);
                is_door_stale = false;
                // increase threshold by constant interval
                open_door_threshold_ms = current_millis + OPEN_DOOR_THRESHOLD_INTERVAL_MS;
            } else {
                if (current_millis >= open_door_threshold_ms) {
                    Serial.println("Door Stale");
                    // we reached the threshold because current time is more than the set
                    // threshold, we'd better close the bulb
                    digitalWrite(RELAY_PIN, false);
                    open_door_threshold_ms = 0;
                    is_door_stale = true;
                } else {
                    Serial.println("Waiting for activity/inactivity");
                    // do nothing, let's continue to count, let's remain it open
                    // we don't want to close the bulb yet
                }
            }
        } else {
            // Let's set the door_last_open_at since we have no value yet
            open_door_threshold_ms = current_millis + OPEN_DOOR_THRESHOLD_INTERVAL_MS;
            digitalWrite(RELAY_PIN, true);
        }
    } else { // door closed / button unpushed
        /** this closed door means two things:
         * 1) the people is inside the room
         * 2) the people is now outside the room
         * lets detect it
         */
        // Serial.println("Door closed!");
        open_door_threshold_ms = 0;
        is_door_stale = false;
        digitalWrite(RELAY_PIN, true);

        if (is_someone_inside) {
            // lets do nothing and keep it as it is, don't cose the bulb/LED/relay
            // just keep tracking the activity
            if (hasPreviousActivity()) {
                Serial.println("We wait up to 5mins");
                // while there's activity, we keep adding 5mins unto it
                // setRemainActiveUntil(SOMEONE_INSIDE_ACTIVE_MS, false);
            } else {
                Serial.println("no act");
                Serial.println(remain_active_until_ms);
                // no activity within 5 mins? why?!
                digitalWrite(RELAY_PIN, false);
            }
        } else {
            // detect if there's activity for the next specific seconds/minutes
            unsigned long current_millis = millis();

            if (is_door_just_closed) {
                if (must_have_activity_until_ms > current_millis) {
                    if (hasPreviousActivity()) {
                        // congrats! someone is inside
                        Serial.println("someone is inside");
                        setRemainActiveUntil(SOMEONE_INSIDE_ACTIVE_MS, true);
                        is_someone_inside = true;
                    } else {
                        Serial.println("Waiting for activity while door is locked!");
                        // must_have_activity_until_ms = 0;
                        // do nothing because current_millis is less than what we are requiring
                        // we're still on the premise of threshold, but still no one is moving inside
                        // they should move, but we'll still keep the LED open for God's sake
                    }
                } else if (must_have_activity_until_ms < current_millis) {
                    if (hasPreviousActivity()) {
                        // haha, someone is just locked inside, we open the
                        digitalWrite(RELAY_PIN, true);
                        setRemainActiveUntil(SOMEONE_INSIDE_ACTIVE_MS, false);
                        is_someone_inside = true;

                        Serial.println("Locked! LED on");
                    } else {
                        // time required lapsed. no one is inside
                        // the door is totally closed, we off the LED and think about nothing, time to rest sensor
                        digitalWrite(RELAY_PIN, false);
                        Serial.println("Timeout! Door is locked and no one is inside");
                    }
                }
            } else {
                // door comes from open and become closed
                Serial.println("Door just closed");
                is_door_just_closed = true;
                // setRemainActiveUntil(MUST_HAVE_ACTIVITY_THRESHOLD_MS, true);
                must_have_activity_until_ms = current_millis + MUST_HAVE_ACTIVITY_THRESHOLD_MS;
                remain_active_until_ms = 0;
            }
        }
    }

    delay(500);
}

/**
 * Will return of there's detected activity for the thresholds of the sensors.
 * e.g. PIR sensor has 5 secs threshold. If there's activity for the last 5 secs in PIR sensor
 * then this function will return true. As of now there are 3 major sensors.
 * Each has it's own threshold
 * DS1 = DS1_THRESHOLD_INTERVAL_MS
 * DS2 = DS2_THRESHOLD_INTERVAL_MS
 * PIR = PIR_THRESHOLD_INTERVAL_MS
 */
bool hasPreviousActivity() {
    // let's check if the current time is reached
    unsigned long current_millis = millis();
    // if the remain_active is higher than current milli. it means we have activity
    // for the past shit shit minutes
    return remain_active_until_ms > current_millis;
}

/**
 * Fetch the current activity, will change the remain_active_until_ms up to a certain time
 * dependes on the threshold
 * e.g. if PIR threshold is 5secs, this function add time to remain_active_until_ms of 5 seconds
 * the current time is 12:00:00, then if there's detected movement in let's say PIR,
 * then the new remain_active_until_ms (which means LED will still remain on) is 12:00:05
 * For sensors threshold look at:
 * DS1 = DS1_THRESHOLD_INTERVAL_MS
 * DS2 = DS2_THRESHOLD_INTERVAL_MS
 * PIR = PIR_THRESHOLD_INTERVAL_MS
 */
void fetchActivity() {
    if (digitalRead(PIN_PIR)) {
        setRemainActiveUntil(PIR_THRESHOLD_INTERVAL_MS, false);
        Serial.println("A Ghost");
    }
    // TODO: change to analogRead
    if (digitalRead(PIN_DS2)) {
        setRemainActiveUntil(DS2_THRESHOLD_INTERVAL_MS, false);
        Serial.println("An Activity in Middle");
    }
    if (digitalRead(PIN_DS1)) {
        setRemainActiveUntil(DS1_THRESHOLD_INTERVAL_MS, false);
        Serial.println("An Activity in Bowl");
    }
}

/**
 * This will mainly set the `remain_active_until_ms` variable. What the variable do is that is the time that the
 * LED will remain active. If the time now is 12:00 and the remain_active_until_ms is 1mins ahead, let's say
 * 12:01, then the LED will turn off at 12:01. Unless forced to change that is used when the door is closed (because
 * we need to set a long time for the door when someone is inside)
 *
 * @param int remain_active_threshold_ms future time that when reached, LED will turn off
 * @param bool force normally, the time will only overridden when the new time is higher than the current time
 * when forced, it will replace the current time without checking if it is more than the current or not.
 */
void setRemainActiveUntil(int remain_active_threshold_ms, bool force = false)
{
    unsigned long new_remain_active_until_ms;
    // this statement is used to bypass the int conversion
    if (remain_active_threshold_ms == SOMEONE_INSIDE_ACTIVE_MS) {
        Serial.println("enter here!");
        new_remain_active_until_ms = millis() + 360000;
    } else {
        new_remain_active_until_ms = millis() + remain_active_threshold_ms;
    }
    if (force) {
        remain_active_until_ms = new_remain_active_until_ms;
        Serial.print("New FORCED remain active: ");
        Serial.println(remain_active_until_ms);
    } else {
        if (remain_active_until_ms > new_remain_active_until_ms) {
            // do nothing, the current time is more than the time you are setting
        } else {
            remain_active_until_ms = new_remain_active_until_ms;
            Serial.print("New remain active: ");
            Serial.println(remain_active_until_ms);
        }
    }
}
