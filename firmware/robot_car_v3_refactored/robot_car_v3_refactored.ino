/*
  Arduino Autonomous Robot Car — Version 3 (Refactored)

  Based on the user's Version 3 sketch. Motor directions, pin assignments,
  PWM settings and maneuver durations are carried over from that version.

  Improvements:
    - Timed maneuvers use millis() rather than delay().
    - Obstacle recovery has a finite 8-second search limit.
    - Ultrasonic pulseIn() has a timeout; missing echoes stop the robot.
    - Only one IR decision is selected at a time (outer sensors first).
    - Control modes make the behavior explicit and easier to debug.

  IMPORTANT: Not verified on the original robot hardware. Confirm wiring,
  IR polarities, wheel rotation and clearances before placing on the floor.
*/

#include <Arduino.h>

// ---- Hardware pins (as provided in Version 3) -------------------------
constexpr uint8_t TRIG_PIN = 13;
constexpr uint8_t ECHO_PIN = 4;

constexpr uint8_t LEFT_IN1 = 8;
constexpr uint8_t LEFT_IN2 = 9;
constexpr uint8_t LEFT_PWM = 10;
constexpr uint8_t RIGHT_IN1 = 7;
constexpr uint8_t RIGHT_IN2 = 6;
constexpr uint8_t RIGHT_PWM = 5;

constexpr uint8_t IR_INNER_RIGHT = 3;
constexpr uint8_t IR_INNER_LEFT = 12;
constexpr uint8_t IR_OUTER_RIGHT = 2;
constexpr uint8_t IR_OUTER_LEFT = 11;

// ---- Version 3 motor calibration ---------------------------------------
constexpr uint8_t PWM_STRAIGHT_LEFT = 150;
constexpr uint8_t PWM_STRAIGHT_RIGHT = 158;
constexpr uint8_t PWM_TURN_LEFT = 125;  // Right motor during left turn
constexpr uint8_t PWM_TURN_RIGHT = 130; // Left motor during right turn

// ---- Safety/measurement policy introduced by this refactor ------------
constexpr float OBSTACLE_THRESHOLD_CM = 15.0f;
constexpr unsigned long ECHO_TIMEOUT_US = 30000UL; // 30 ms max wait
constexpr unsigned long SONAR_INTERVAL_MS = 80UL;
constexpr unsigned long SEARCH_TIMEOUT_MS = 8000UL; // Changed from unbounded loop

// ---- Motor commands ----------------------------------------------------
enum Motion : uint8_t { STOP, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT };

void drive(Motion motion) {
  switch (motion) {
    case FORWARD:
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, HIGH);
      digitalWrite(RIGHT_IN1, HIGH);
      digitalWrite(RIGHT_IN2, LOW);
      analogWrite(LEFT_PWM, PWM_STRAIGHT_LEFT);
      analogWrite(RIGHT_PWM, PWM_STRAIGHT_RIGHT);
      break;
    case BACKWARD:
      digitalWrite(LEFT_IN1, HIGH);
      digitalWrite(LEFT_IN2, LOW);
      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, HIGH);
      analogWrite(LEFT_PWM, PWM_STRAIGHT_LEFT);
      analogWrite(RIGHT_PWM, PWM_STRAIGHT_RIGHT);
      break;
    case TURN_LEFT:
      analogWrite(LEFT_PWM, 0);
      digitalWrite(RIGHT_IN1, HIGH);
      digitalWrite(RIGHT_IN2, LOW);
      analogWrite(RIGHT_PWM, PWM_TURN_LEFT);
      break;
    case TURN_RIGHT:
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, HIGH);
      analogWrite(LEFT_PWM, PWM_TURN_RIGHT);
      analogWrite(RIGHT_PWM, 0);
      break;
    case STOP:
    default:
      analogWrite(LEFT_PWM, 0);
      analogWrite(RIGHT_PWM, 0);
      break;
  }
}

// ---- Timed sequences (original Version 3 timings) ---------------------
struct TimedStep {
  Motion motion;
  uint16_t durationMs;
};

const TimedStep OBSTACLE_STEPS[] = {
  {STOP, 1000}, {TURN_LEFT, 700}, {STOP, 1000},
  {FORWARD, 500}, {STOP, 1000}, {TURN_RIGHT, 750}, {STOP, 1000}
};
const TimedStep OUTER_RIGHT_STEPS[] = {
  {BACKWARD, 30}, {TURN_RIGHT, 300}, {STOP, 10}
};
const TimedStep OUTER_LEFT_STEPS[] = {
  {BACKWARD, 30}, {TURN_LEFT, 500}, {STOP, 10}
};
const TimedStep INNER_BOTH_HIGH_STEPS[] = {
  {STOP, 100}, {FORWARD, 100}
};
const TimedStep INNER_LEFT_LOW_STEPS[] = {
  {TURN_LEFT, 30}, {STOP, 10}
};
const TimedStep INNER_RIGHT_LOW_STEPS[] = {
  {TURN_RIGHT, 30}, {STOP, 10}
};
const TimedStep INNER_BOTH_LOW_STEPS[] = {
  {STOP, 100}, {BACKWARD, 50}
};

template <size_t N>
constexpr uint8_t stepCount(const TimedStep (&)[N]) {
  return static_cast<uint8_t>(N);
}

enum ControllerMode : uint8_t {
  FOLLOW_LINE,
  EXECUTE_STEPS,
  SEARCH_FOR_LINE,
  SONAR_FAULT,
  SEARCH_TIMEOUT_FAULT
};

ControllerMode mode = FOLLOW_LINE;
const TimedStep* activeSteps = nullptr;
uint8_t activeStepCount = 0;
uint8_t activeStepIndex = 0;
unsigned long stepStartedAt = 0;
bool searchAfterSteps = false;
unsigned long searchStartedAt = 0;
bool searchForward = true;
unsigned long searchPhaseStartedAt = 0;
unsigned long lastSonarAt = 0;

void startTimedSteps(const TimedStep* steps, uint8_t count,
                     bool searchAfter, unsigned long now) {
  activeSteps = steps;
  activeStepCount = count;
  activeStepIndex = 0;
  searchAfterSteps = searchAfter;
  stepStartedAt = now;
  mode = EXECUTE_STEPS;
  drive(activeSteps[0].motion);
}

void beginLineSearch(unsigned long now) {
  mode = SEARCH_FOR_LINE;
  searchStartedAt = now;
  searchPhaseStartedAt = now;
  searchForward = true;
  drive(FORWARD); // Version 3: forward 100 ms, stop 500 ms, repeat
  Serial.println(F("Searching for line after obstacle avoidance"));
}

void updateTimedSteps(unsigned long now) {
  if (now - stepStartedAt < activeSteps[activeStepIndex].durationMs) return;
  ++activeStepIndex;
  if (activeStepIndex >= activeStepCount) {
    drive(STOP);
    if (searchAfterSteps) {
      beginLineSearch(now);
    } else {
      mode = FOLLOW_LINE;
    }
    return;
  }
  stepStartedAt = now;
  drive(activeSteps[activeStepIndex].motion);
}

void updateLineSearch(unsigned long now) {
  // Original Version 3 continued while BOTH inner sensors were LOW.
  // Here, either HIGH reading exits the search. Verify polarity on hardware.
  if (digitalRead(IR_INNER_LEFT) == HIGH ||
      digitalRead(IR_INNER_RIGHT) == HIGH) {
    drive(STOP);
    mode = FOLLOW_LINE;
    Serial.println(F("Line detected; returning to line following"));
    return;
  }

  if (now - searchStartedAt >= SEARCH_TIMEOUT_MS) {
    drive(STOP);
    mode = SEARCH_TIMEOUT_FAULT; // Latched until reboot, by design
    Serial.println(F("FAULT: line not found within search timeout; reset required"));
    return;
  }

  const unsigned long phaseMs = searchForward ? 100UL : 500UL;
  if (now - searchPhaseStartedAt >= phaseMs) {
    searchForward = !searchForward;
    searchPhaseStartedAt = now;
    drive(searchForward ? FORWARD : STOP);
  }
}

// Returns false if pulseIn timed out. No echo is NOT treated as zero distance.
bool measureDistanceCm(float& cm) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const unsigned long durationUs = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (durationUs == 0) return false;
  cm = durationUs * 0.0343f * 0.5f;
  return true;
}

void chooseLineAction(unsigned long now) {
  // A priority chain prevents multiple movement plans in one iteration.
  const bool outerRight = digitalRead(IR_OUTER_RIGHT) == HIGH;
  const bool outerLeft = digitalRead(IR_OUTER_LEFT) == HIGH;
  const bool innerLeft = digitalRead(IR_INNER_LEFT) == HIGH;
  const bool innerRight = digitalRead(IR_INNER_RIGHT) == HIGH;

  if (outerRight && !outerLeft) {
    startTimedSteps(OUTER_RIGHT_STEPS, stepCount(OUTER_RIGHT_STEPS), false, now);
  } else if (!outerRight && outerLeft) {
    startTimedSteps(OUTER_LEFT_STEPS, stepCount(OUTER_LEFT_STEPS), false, now);
  } else if (innerLeft && innerRight) {
    startTimedSteps(INNER_BOTH_HIGH_STEPS, stepCount(INNER_BOTH_HIGH_STEPS), false, now);
  } else if (!innerLeft && innerRight) {
    startTimedSteps(INNER_LEFT_LOW_STEPS, stepCount(INNER_LEFT_LOW_STEPS), false, now);
  } else if (innerLeft && !innerRight) {
    startTimedSteps(INNER_RIGHT_LOW_STEPS, stepCount(INNER_RIGHT_LOW_STEPS), false, now);
  } else {
    startTimedSteps(INNER_BOTH_LOW_STEPS, stepCount(INNER_BOTH_LOW_STEPS), false, now);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(IR_INNER_RIGHT, INPUT);
  pinMode(IR_INNER_LEFT, INPUT);
  pinMode(IR_OUTER_RIGHT, INPUT);
  pinMode(IR_OUTER_LEFT, INPUT);

  digitalWrite(TRIG_PIN, LOW);
  drive(STOP);
  lastSonarAt = millis() - SONAR_INTERVAL_MS; // Read at startup
  Serial.println(F("Robot car Version 3 refactored: ready"));
}

void loop() {
  const unsigned long now = millis();

  if (mode == EXECUTE_STEPS) {
    updateTimedSteps(now);
    return;
  }
  if (mode == SEARCH_FOR_LINE) {
    updateLineSearch(now);
    return;
  }
  if (mode == SEARCH_TIMEOUT_FAULT) {
    drive(STOP);
    return;
  }

  // Sonar is sampled during line-follow mode and while recovering from a
  // sensor fault. During timed steps it is not sampled (documented limit).
  if (now - lastSonarAt >= SONAR_INTERVAL_MS) {
    lastSonarAt = now;
    float distanceCm = 0.0f;
    if (!measureDistanceCm(distanceCm)) {
      if (mode != SONAR_FAULT) {
        Serial.println(F("FAULT: ultrasonic echo missing; motors stopped"));
      }
      drive(STOP);
      mode = SONAR_FAULT;
      return;
    }

    if (mode == SONAR_FAULT) {
      mode = FOLLOW_LINE;
      Serial.println(F("Ultrasonic echo restored"));
    }
    if (distanceCm < OBSTACLE_THRESHOLD_CM) {
      Serial.println(F("Obstacle detected; running avoidance sequence"));
      startTimedSteps(OBSTACLE_STEPS, stepCount(OBSTACLE_STEPS), true, millis());
      return;
    }
  }

  if (mode == SONAR_FAULT) {
    drive(STOP);
    return;
  }
  chooseLineAction(millis());
}
