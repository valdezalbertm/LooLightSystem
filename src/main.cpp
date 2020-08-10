#include <Arduino.h>

/**
 * ATTiny85
 * SRAM: 512 Bytes
 * Flash Memory: 8kb
 */

const byte DOOR_PIN = 2; // magnetic reed sensor // digital
const byte PIN_DS2 = 3;
const byte PIN_DS1 = 4;
const byte PIN_TRIG = 5;
const byte PIN_PIR = 6; // digital
const byte RELAY_PIN = 7; //digital

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
unsigned long ds2_current_value, ds2_duration, ds2_val_cm;
unsigned long ds2_next_sense_activate_ms;

// Common variable in each Distance Sensor
const byte DS_THRESHOLD_CM = 4;
const byte DS_MEASURE_MIN_LEN_CM = 25;
const int DS_MEASURE_MAX_LEN_CM = 375;
const int NEXT_DS_ACTIVATE_SECS = 1;
bool reset_ds = false;

/*** DISTANCE SENSOR 1 - BOWL ***/
int pin_ds1_val; // analog
const unsigned int DS1_THRESHOLD_INTERVAL_MS = 20000;
unsigned long ds1_current_value, ds1_duration, ds1_val_cm;
unsigned long ds1_next_sense_activate_ms;

/*** PIR SENSOR ***/
bool has_pir_detected; // digital
const unsigned int PIR_THRESHOLD_INTERVAL_MS = 5000;
unsigned int pir_last_activity_ms;

bool is_led_light; // digital

bool has_activity = false;

// time to remain the LED/relay/bulb until the millis reached this time
// specific sensors has each threshold
unsigned long remain_active_until_ms;

unsigned long must_have_activity_until_ms;
unsigned int MUST_HAVE_ACTIVITY_THRESHOLD_MS = 8000;

unsigned long someone_inside_remain_active_until_ms;
bool is_someone_inside;

const unsigned int HIGH_SOMEONE_INSIDE_ACTIVE_MIN = 5; // this is 6 minutes
const unsigned int LOW_SOMEONE_INSIDE_ACTIVE_MIN = 1; // this is 1 minute
bool is_activated_from_low = true;

void activateSomeoneInsideTimer(bool type);
void doorClosed();
void doorOpen();
void fetchActivity();
void fetchActivityDistanceSensor();
void fetchActivityDistanceSensor2();
bool hasPreviousActivity();
bool isDSMeasureWithinThreshold(unsigned long ds_measure, unsigned int ds_current_value);
void requireActivityWithinSecs();
void resetActivity();
void resetRequiringActivity();
void setBulb(bool request_on);
void setRemainActiveUntil(int remain_active_threshold_ms, bool force);
void triggerDistanceSensor();

// cppcheck-suppress unusedFunction
void setup()
{
    Serial.begin(9600);

    is_door_stale = false;
    remain_active_until_ms = millis() + 5000;
    must_have_activity_until_ms = 0;
    is_door_just_closed = false;
    is_someone_inside = false;
    ds1_current_value = 0;
    ds2_current_value = 0;
    is_activated_from_low = false;

    pinMode(RELAY_PIN, OUTPUT);
    pinMode(PIN_PIR, INPUT);
    pinMode(PIN_DS1, INPUT);
    pinMode(PIN_DS2, INPUT);
    pinMode(PIN_TRIG, OUTPUT);
}

// cppcheck-suppress unusedFunction
void loop()
{
    fetchActivity();

    is_door_open = digitalRead(DOOR_PIN);
    if (is_door_open) {
        // Button is pushed
        doorOpen();
    } else { // door closed / button unpushed
        /** this closed door means two things:
         * 1) the people is inside the room
         * 2) the people is now outside the room
         * lets detect it
         */
        Serial.println("Door closed!");
        doorClosed();
    }

    // delay(200);
}

void doorOpen() {
    // Serial.println("Door is now/still open");
    is_door_just_closed = false;
    is_someone_inside = false;
    someone_inside_remain_active_until_ms = 0; // we end this because no one is inside now

    unsigned long current_millis = millis();

    if (is_door_stale) {
        if (hasPreviousActivity()) {
            Serial.println("Someone got inside after a long time!");
            // there's a sudden activity, someone got inside after a long time
            is_door_stale = false;
            setBulb(HIGH);
        } else {
            Serial.println("Door remains stale.");
            // do nothing, door is stale, and no activity
            // let's keep relay closed
        }
    } else if (open_door_threshold_ms > 0) {
        if (hasPreviousActivity()) {
            // Serial.println("Door is open, and someone inside, maybe washing feet");
            setBulb(HIGH);
            is_door_stale = false;
            // increase threshold by constant interval
            open_door_threshold_ms = current_millis + OPEN_DOOR_THRESHOLD_INTERVAL_MS;
        } else {
            if (current_millis >= open_door_threshold_ms) {
                Serial.println("Door Stale");
                // we reached the threshold because current time is more than the set
                // threshold, we'd better close the bulb
                setBulb(LOW);
                open_door_threshold_ms = 0;
                is_door_stale = true;
            } else {
                // Serial.println("Waiting for activity/inactivity");
                // do nothing, let's continue to count, let's remain it open
                // we don't want to close the bulb yet
            }
        }
    } else {
        // Let's set the door_last_open_at since we have no value yet
        open_door_threshold_ms = current_millis + OPEN_DOOR_THRESHOLD_INTERVAL_MS;
        setBulb(HIGH);
    }
}

void doorClosed() {
    open_door_threshold_ms = 0;
    is_door_stale = false;

    unsigned long current_millis = millis();

    if (is_someone_inside) {
        // lets do nothing and keep it as it is, don't cose the bulb/LED/relay
        // just keep tracking the activity

        // it means that someone is inside already, and we'll check if they are moving,
        // either comes from HIGH or LOW
        if (hasPreviousActivity()) {
            Serial.println("There's moving inside. We extend 5mins more!");
            setBulb(HIGH);
            if (is_activated_from_low) {
                is_activated_from_low = !is_activated_from_low;
                delay(1000);
            } else {
                // while there's activity, we keep adding 5mins unto it
                activateSomeoneInsideTimer(HIGH);
            }
        } else {
            if (someone_inside_remain_active_until_ms > current_millis) {
                // We continue countdown until we reached like 1min (locked) or 5mins (truly action)
                // Serial.println("Counting down 5 minutes");
                Serial.print("no act, we'll still active until: ");
                // Serial.println(someone_inside_remain_active_until_ms);
            } else {
                Serial.println("No activity in 5 mins?! Why?!");
                // no activity within 5 mins? why?!
                setBulb(LOW);
            }
        }
    } else {
        // detect if there's activity for the next specific seconds/minutes
        if (is_door_just_closed) {
            if (must_have_activity_until_ms >= current_millis) {
                if (hasPreviousActivity()) {
                    // congrats! someone is inside
                    // Serial.println("someone is inside");
                    if (is_activated_from_low) {
                        is_activated_from_low = !is_activated_from_low;
                    } else {
                        activateSomeoneInsideTimer(HIGH);
                    }
                    is_someone_inside = true;
                    resetRequiringActivity(); // we're not requiring any act anymore because we already got it
                } else {
                    // Serial.println("Waiting for activity while door is locked!");
                    // do nothing because current_millis is less than what we are requiring
                    // we're still on the premise of threshold, but still no one is moving inside
                    // they should move, but we'll still keep the LED open for God's sake
                }
            } else if (must_have_activity_until_ms < current_millis && !is_someone_inside) {
                if (hasPreviousActivity()) { // this statement is used to catch when someone inside and just moved
                    Serial.println("Locked! LED on");
                    // haha, someone is just locked inside, we open the
                    setBulb(HIGH);
                    activateSomeoneInsideTimer(LOW);
                    is_someone_inside = true;
                    resetRequiringActivity();
                    is_activated_from_low = true;
                    delay(5000); // since we are already high, we can delay things and
                        // check later, we don't want to detect any changes for the next changes
                } else {
                    // time required lapsed. no one is inside
                    // the door is totally closed, we off the LED and think about nothing, time to rest sensor
                    setBulb(LOW);
                    Serial.println("Timeout! Door is locked and no one is inside");
                }
            }
        } else {
            // door comes from open and become closed
            // Serial.println("Door just closed");
            is_door_just_closed = true;
            requireActivityWithinSecs();
            digitalWrite(RELAY_PIN, true);
            resetActivity();
        }
    }
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
    // if the remain_active is higher than current milli. it means we have activity
    // for the past shit shit minutes
    return remain_active_until_ms > millis();
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
    fetchActivityDistanceSensor();

    if (digitalRead(PIN_PIR)) {
        setRemainActiveUntil(PIR_THRESHOLD_INTERVAL_MS, false);
        Serial.println("PIR detected ghost!");
    }

    fetchActivityDistanceSensor2();
}

void fetchActivityDistanceSensor() {
    if (millis() < ds1_next_sense_activate_ms) {
        // not time yet MOFO, return
        return;
    }
    Serial.println("DS1 running!");
    triggerDistanceSensor();

    ds1_duration = pulseIn(PIN_DS1, HIGH);
    ds1_val_cm = (ds1_duration / 2) / 29.1; // Convert the time into a distance, divide by 29.1 or multiply by 0.0343

    if (isDSMeasureWithinThreshold(ds1_val_cm, ds1_current_value)) {
        Serial.print("CM:");
        Serial.print(ds1_val_cm);
        // Detected change in value
        Serial.print(",");
        Serial.print(", current value: ");
        Serial.println(ds1_current_value);
        setRemainActiveUntil(DS1_THRESHOLD_INTERVAL_MS, false);
        Serial.println("An Activity in Bowl");

        ds1_current_value  = ds1_val_cm;
    }

    ds2_next_sense_activate_ms = millis() + (NEXT_DS_ACTIVATE_SECS * 400);
    ds1_next_sense_activate_ms = millis() + (NEXT_DS_ACTIVATE_SECS * 800);
}

void fetchActivityDistanceSensor2() {
    if (millis() < ds2_next_sense_activate_ms) {
        // not time yet MOFO, return
        return;
    }

    Serial.println("DS2 running!");
    triggerDistanceSensor();

    ds2_duration = pulseIn(PIN_DS2, HIGH);
    ds2_val_cm = (ds2_duration / 2) / 29.1; // Convert the time into a distance, divide by 29.1 or multiply by 0.0343

    if (isDSMeasureWithinThreshold(ds2_val_cm, ds2_current_value)) {
        Serial.print("CM:");
        Serial.print(ds2_val_cm);
        // Detected change in value
        Serial.print(",");
        Serial.print(", current value: ");
        Serial.println(ds2_current_value);
        setRemainActiveUntil(DS2_THRESHOLD_INTERVAL_MS, false);
        Serial.println("An Activity in Middle");

        ds2_current_value  = ds2_val_cm;
    }

    ds1_next_sense_activate_ms = millis() + (NEXT_DS_ACTIVATE_SECS * 400);
    ds2_next_sense_activate_ms = millis() + (NEXT_DS_ACTIVATE_SECS * 800);
}

/**
 * We want to make sure that the distance sensor meaure is within 10-350cm
 * and new value is within range to detect as new changes
 */
bool isDSMeasureWithinThreshold(unsigned long ds_measure, unsigned int ds_current_value) {
    Serial.print(ds_measure);
    Serial.print(" . ");
    Serial.println(ds_current_value);
    bool is_within_range = ds_measure > DS_MEASURE_MIN_LEN_CM && ds_measure < DS_MEASURE_MAX_LEN_CM;
    bool is_significant_change = ds_measure > (ds_current_value + DS_THRESHOLD_CM) ||
        ds_measure < (ds_current_value - DS_THRESHOLD_CM);

    return is_within_range && is_significant_change;
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
    unsigned long new_remain_active_until_ms = millis() + remain_active_threshold_ms;
    if (force) {
        remain_active_until_ms = new_remain_active_until_ms;
        // Serial.print("New FORCED remain active: ");
        // Serial.println(remain_active_until_ms);
    } else {
        if (remain_active_until_ms > new_remain_active_until_ms) {
            // do nothing, the current time is more than the time you are setting
        } else {
            remain_active_until_ms = new_remain_active_until_ms;
            // Serial.print("New remain active: ");
            // Serial.println(remain_active_until_ms);
        }
    }
}

/**
 * Update the some_inside_remain_active var, which will not turn off the light until it reached
 * this time.
 */
void activateSomeoneInsideTimer(bool type)
{
    unsigned long addtl_someone_inside_active_min;
    if (type) {
        addtl_someone_inside_active_min = HIGH_SOMEONE_INSIDE_ACTIVE_MIN;
        Serial.println("Activated Someone Inside HIGH");
    } else {
        addtl_someone_inside_active_min = LOW_SOMEONE_INSIDE_ACTIVE_MIN;
        Serial.println("Activated Someone Inside LOW");
    }
    // someone_inside_remain_active_until_ms = millis() + (addtl_someone_inside_active_min  * 60 * 1000);
    someone_inside_remain_active_until_ms = millis() + (addtl_someone_inside_active_min  * 1 * 500);
}

void setBulb(bool request_on) {
    bool relay_val = digitalRead(RELAY_PIN);
    if (request_on && relay_val == LOW) {
        digitalWrite(RELAY_PIN, HIGH);
    } else if (!request_on && relay_val == HIGH) {
        digitalWrite(RELAY_PIN, LOW);
    }
}

void resetActivity() {
    remain_active_until_ms = 0;
    delay(3000); // this should be set depends on the PIR reset time
}

void requireActivityWithinSecs() {
    must_have_activity_until_ms = millis() + MUST_HAVE_ACTIVITY_THRESHOLD_MS;
}

void resetRequiringActivity() {
    must_have_activity_until_ms = 0;
}

void triggerDistanceSensor() {
    Serial.println("----------------------------------------------");
    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(3000); // default 5
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
}
