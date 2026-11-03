# Version 3 Refactor — Hardware Validation Checklist

This checklist describes tests to run; it does **not** imply they were passed.

## Initial safety check

- [ ] Confirm Arduino board model and motor-driver voltage/current limits.
- [ ] Compare actual TRIG/ECHO/IR/GPIO/PWM wiring with the README mapping.
- [ ] Lift wheels off the floor before first upload and motor tests.
- [ ] Ensure emergency power removal is accessible.

## IO and motor direction

- [ ] Confirm PWM 150/158 drives forward on both wheels with V3 wiring.
- [ ] Confirm backward reverses both wheels.
- [ ] Confirm turning left holds left wheel at 0 and drives right wheel.
- [ ] Confirm turning right holds right wheel at 0 and drives left wheel.
- [ ] Confirm both PWM outputs are zero when stopped.

## IR sensor logic

- [ ] Record HIGH/LOW on black and white for each of D2, D3, D11, D12.
- [ ] Validate right outer correction (D2=1, D11=0).
- [ ] Validate left outer correction (D2=0, D11=1).
- [ ] Validate all four inner IR patterns on D12 and D3.
- [ ] Confirm equal outer states do not cause unwanted outer corrections.

## Ultrasonic and obstacle recovery

- [ ] Validate distance measurement against a known object at 10, 15 and 30 cm.
- [ ] Disconnect/obscure echo safely and confirm motors stop on missing echo.
- [ ] Restore echo and confirm control can resume.
- [ ] Confirm 15 cm obstacle trigger and correct turn sequence.
- [ ] Confirm repeated forward/stop search behavior.
- [ ] Confirm search stops when either inner IR reads HIGH.
- [ ] Confirm 8-second search timeout stops motors and requires reset.
- [ ] Confirm motor/sensor operations are safe on the target floor and course.

## Notes

Record date, board revision, motor-driver model, sensor module polarity, battery voltage, actual pass/fail, and observed deviations here before claiming hardware validation.
