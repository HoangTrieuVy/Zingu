#include "MotorDriver.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"

namespace {

constexpr uint8_t kPwmChannelLeft = 0;
constexpr uint8_t kPwmChannelRight = 1;
constexpr float kPwmMaxCount = (1 << cfg::kMotorPwmResolutionBits) - 1;

// Encoder counters live at file scope because the ISRs need them. Only the ISRs
// write; the control loop reads a snapshot taken with interrupts masked.
volatile int32_t g_left_count = 0;
volatile int32_t g_right_count = 0;

void IRAM_ATTR onLeftEncoderA() {
  // Quadrature decode on one edge: B's level at the A edge gives the direction.
  const bool b = digitalRead(cfg::kEncoderLeftBPin);
  g_left_count += b ? 1 : -1;
}

void IRAM_ATTR onRightEncoderA() {
  const bool b = digitalRead(cfg::kEncoderRightBPin);
  // Mirrored motor mounting: the right wheel turns the opposite way for the
  // same forward motion, so its count is negated here rather than everywhere
  // downstream.
  g_right_count += b ? -1 : 1;
}

float clampDuty(float duty) {
  if (duty > 1.0f) return 1.0f;
  if (duty < -1.0f) return -1.0f;
  return duty;
}

}  // namespace

void MotorDriver::begin() {
  pinMode(cfg::kMotorLeftIn1Pin, OUTPUT);
  pinMode(cfg::kMotorLeftIn2Pin, OUTPUT);
  pinMode(cfg::kMotorRightIn1Pin, OUTPUT);
  pinMode(cfg::kMotorRightIn2Pin, OUTPUT);
  pinMode(cfg::kMotorStandbyPin, OUTPUT);

  ledcSetup(kPwmChannelLeft, cfg::kMotorPwmFrequencyHz,
            cfg::kMotorPwmResolutionBits);
  ledcSetup(kPwmChannelRight, cfg::kMotorPwmFrequencyHz,
            cfg::kMotorPwmResolutionBits);
  ledcAttachPin(cfg::kMotorLeftPwmPin, kPwmChannelLeft);
  ledcAttachPin(cfg::kMotorRightPwmPin, kPwmChannelRight);

  pinMode(cfg::kEncoderLeftAPin, INPUT);
  pinMode(cfg::kEncoderLeftBPin, INPUT);
  pinMode(cfg::kEncoderRightAPin, INPUT);
  pinMode(cfg::kEncoderRightBPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(cfg::kEncoderLeftAPin),
                  onLeftEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(cfg::kEncoderRightAPin),
                  onRightEncoderA, RISING);

  digitalWrite(cfg::kMotorStandbyPin, HIGH);  // take the H-bridge out of standby
  coast();
}

void MotorDriver::applySide(uint8_t pwm_channel, uint8_t in1_pin,
                            uint8_t in2_pin, float duty) {
  duty = clampDuty(duty);
  const float magnitude = fabsf(duty);

  if (magnitude < 1e-3f) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
    ledcWrite(pwm_channel, 0);
    return;
  }

  // Map [0,1] onto [deadband,1] so the smallest non-zero command still moves
  // the wheel. Without this the integrator has to wind up through the flat spot
  // every time the robot crosses vertical, which reads as a wobble.
  const float scaled =
      cfg::kMotorDeadbandDuty + magnitude * (1.0f - cfg::kMotorDeadbandDuty);

  digitalWrite(in1_pin, duty > 0.0f ? HIGH : LOW);
  digitalWrite(in2_pin, duty > 0.0f ? LOW : HIGH);
  ledcWrite(pwm_channel, static_cast<uint32_t>(scaled * kPwmMaxCount));
}

void MotorDriver::setDuty(float left, float right) {
  applySide(kPwmChannelLeft, cfg::kMotorLeftIn1Pin, cfg::kMotorLeftIn2Pin, left);
  applySide(kPwmChannelRight, cfg::kMotorRightIn1Pin, cfg::kMotorRightIn2Pin,
            right);
}

void MotorDriver::coast() {
  ledcWrite(kPwmChannelLeft, 0);
  ledcWrite(kPwmChannelRight, 0);
  digitalWrite(cfg::kMotorLeftIn1Pin, LOW);
  digitalWrite(cfg::kMotorLeftIn2Pin, LOW);
  digitalWrite(cfg::kMotorRightIn1Pin, LOW);
  digitalWrite(cfg::kMotorRightIn2Pin, LOW);
}

void MotorDriver::brake() {
  // Both inputs high shorts the winding through the bridge.
  digitalWrite(cfg::kMotorLeftIn1Pin, HIGH);
  digitalWrite(cfg::kMotorLeftIn2Pin, HIGH);
  digitalWrite(cfg::kMotorRightIn1Pin, HIGH);
  digitalWrite(cfg::kMotorRightIn2Pin, HIGH);
  ledcWrite(kPwmChannelLeft, static_cast<uint32_t>(kPwmMaxCount));
  ledcWrite(kPwmChannelRight, static_cast<uint32_t>(kPwmMaxCount));
}

void MotorDriver::update(float dt) {
  noInterrupts();
  const int32_t left = g_left_count;
  const int32_t right = g_right_count;
  interrupts();

  const int32_t d_left = left - last_left_count_;
  const int32_t d_right = right - last_right_count_;
  last_left_count_ = left;
  last_right_count_ = right;

  const float mean_counts = (d_left + d_right) * 0.5f;
  const float revolutions = mean_counts / cfg::kEncoderCountsPerWheelRev;

  wheel_position_rev_ += revolutions;
  wheel_velocity_rps_ = revolutions / dt;
}

void MotorDriver::resetOdometry() {
  noInterrupts();
  last_left_count_ = g_left_count;
  last_right_count_ = g_right_count;
  interrupts();

  wheel_position_rev_ = 0.0f;
  wheel_velocity_rps_ = 0.0f;
}
