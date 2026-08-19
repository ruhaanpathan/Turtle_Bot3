#!/usr/bin/env python3
"""
Quick hardware test — independent of ROS.
Tests motor output and encoder input via lgpio on Pi 5.
Run: python3 test_gpio.py
"""
import lgpio
import time

CHIP = 4  # Pi 5: gpiochip4 = pinctrl-rp1 (40-pin header)

# Motor pins (left motor)
PWM_PIN = 18
IN1_PIN = 23
IN2_PIN = 24

# Encoder pins (left encoder)
ENC_A = 17
ENC_B = 27

tick_count = 0
handle = None  # Global handle for callback

def encoder_callback(chip, gpio, level, timestamp):
    global tick_count, handle
    if level == 2:  # watchdog timeout
        return
    b = lgpio.gpio_read(handle, ENC_B)  # Must use handle, NOT chip number
    if level == b:
        tick_count += 1
    else:
        tick_count -= 1

def main():
    global tick_count, handle
    handle = lgpio.gpiochip_open(CHIP)
    print(f"GPIO chip {CHIP} opened (handle={handle})")

    # --- Test 1: Motor output ---
    print("\n=== TEST 1: Motor Spin ===")
    lgpio.gpio_claim_output(handle, IN1_PIN)
    lgpio.gpio_claim_output(handle, IN2_PIN)
    lgpio.gpio_claim_output(handle, PWM_PIN)

    lgpio.gpio_write(handle, IN1_PIN, 1)
    lgpio.gpio_write(handle, IN2_PIN, 0)
    lgpio.tx_pwm(handle, PWM_PIN, 1000, 50)
    print("Motor spinning FORWARD at 50%...")
    time.sleep(2)

    lgpio.gpio_write(handle, IN1_PIN, 0)
    lgpio.gpio_write(handle, IN2_PIN, 0)
    lgpio.tx_pwm(handle, PWM_PIN, 1000, 0)
    print("Motor stopped.\n")
    time.sleep(0.5)

    lgpio.gpio_free(handle, PWM_PIN)
    lgpio.gpio_free(handle, IN1_PIN)
    lgpio.gpio_free(handle, IN2_PIN)

    # --- Test 2: Encoder only (manual spin) ---
    print("=== TEST 2: Encoder (manual spin) ===")
    print("Spin the motor shaft BY HAND for 5 seconds...\n")

    lgpio.gpio_claim_input(handle, ENC_B)
    lgpio.gpio_claim_alert(handle, ENC_A, lgpio.BOTH_EDGES)
    cb = lgpio.callback(handle, ENC_A, lgpio.BOTH_EDGES, encoder_callback)

    tick_count = 0
    for i in range(5):
        time.sleep(1)
        print(f"  {i+1}s: ticks = {tick_count}")

    cb.cancel()
    lgpio.gpio_free(handle, ENC_A)
    lgpio.gpio_free(handle, ENC_B)

    if tick_count != 0:
        print(f"\n✅ Encoder works! Total ticks: {tick_count}")
    else:
        print(f"\n⚠️  No ticks from manual spin.")

    # --- Test 3: Motor + Encoder together ---
    print("\n=== TEST 3: Motor + Encoder Together ===")
    print("Motor will spin while we count encoder ticks.\n")

    # Re-claim encoder pins
    lgpio.gpio_claim_input(handle, ENC_B)
    lgpio.gpio_claim_alert(handle, ENC_A, lgpio.BOTH_EDGES)
    cb = lgpio.callback(handle, ENC_A, lgpio.BOTH_EDGES, encoder_callback)

    # Re-claim motor pins
    lgpio.gpio_claim_output(handle, IN1_PIN)
    lgpio.gpio_claim_output(handle, IN2_PIN)
    lgpio.gpio_claim_output(handle, PWM_PIN)

    tick_count = 0

    # Spin motor forward
    lgpio.gpio_write(handle, IN1_PIN, 1)
    lgpio.gpio_write(handle, IN2_PIN, 0)
    lgpio.tx_pwm(handle, PWM_PIN, 1000, 40)

    for i in range(5):
        time.sleep(1)
        print(f"  {i+1}s: ticks = {tick_count}")

    # Stop motor
    lgpio.gpio_write(handle, IN1_PIN, 0)
    lgpio.gpio_write(handle, IN2_PIN, 0)
    lgpio.tx_pwm(handle, PWM_PIN, 1000, 0)

    cb.cancel()

    if tick_count != 0:
        print(f"\n✅ Motor + Encoder working! Total ticks: {tick_count}")
    else:
        print("\n❌ Motor spins but encoder reads 0.")
        print("   Check encoder wiring:")
        print("   Yellow (A) → GPIO17")
        print("   Green  (B) → GPIO27")
        print("   Blue       → Pi 3.3V")
        print("   Black      → Pi GND")

    # Also do a raw pin read test
    print("\n=== Raw Pin Read ===")
    lgpio.gpio_free(handle, PWM_PIN)
    lgpio.gpio_free(handle, IN1_PIN)
    lgpio.gpio_free(handle, IN2_PIN)
    lgpio.gpio_free(handle, ENC_A)
    lgpio.gpio_free(handle, ENC_B)

    lgpio.gpio_claim_input(handle, ENC_A)
    lgpio.gpio_claim_input(handle, ENC_B)
    for i in range(10):
        a = lgpio.gpio_read(handle, ENC_A)
        b = lgpio.gpio_read(handle, ENC_B)
        print(f"  A(GPIO17)={a}  B(GPIO27)={b}")
        time.sleep(0.1)

    lgpio.gpiochip_close(handle)

if __name__ == "__main__":
    main()
