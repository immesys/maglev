

#define PWM_MAX 2048

pid_controller_t pid;

double halleffect() {

}
void set_pwm(double v) {

}
void setup() {
  // put your setup code here, to run once:
  double initial = halleffect();
  pid_set_output_clamp(&pid, 0, 1);
  pid_set_enabled(&pid, 1, initial);
}

void loop() {
  // put your main code here, to run repeatedly:
  double currentVal = halleffect();
  double newpwm = pid_step(&pid, currentVal);
  set_pwm(newpwm);
}
