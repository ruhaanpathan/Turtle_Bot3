#!/usr/bin/env python3
"""
Standalone Motor Test Script for Smartelex 15D Dual DC Driver on Raspberry Pi 5.
Uses lgpio library to directly drive PWM and DIR pins.

Hardware Pinout (BCM numbering):
  Left Motor:  PWM1 = GPIO 18, DIR1 = GPIO 23
  Right Motor: PWM2 = GPIO 19, DIR2 = GPIO 24

Usage:
  python3 test_motors.py
"""

import time
import lgpio

# BCM Pins for Smartelex 15D
LEFT_PWM = 18
LEFT_DIR = 23
RIGHT_PWM = 19
RIGHT_DIR = 24

PWM_FREQ = 1000  # 1 kHz standard software PWM for lgpio on Raspberry Pi 5


def open_gpio():
    for chip_num in [4, 0]:
        try:
            h = lgpio.gpiochip_open(chip_num)
            print(f"✅ Opened gpiochip{chip_num} (handle={h})")
            return h
        except Exception as e:
            print(f"Could not open gpiochip{chip_num}: {e}")
    raise RuntimeError("Failed to open any GPIO chip!")


def setup_pins(h):
    lgpio.gpio_claim_output(h, LEFT_PWM, 0)
    lgpio.gpio_claim_output(h, LEFT_DIR, 0)
    lgpio.gpio_claim_output(h, RIGHT_PWM, 0)
    lgpio.gpio_claim_output(h, RIGHT_DIR, 0)
    print("✅ Motor pins claimed as outputs.")


def set_left(h, speed_percent, forward=True):
    dir_val = 1 if forward else 0
    lgpio.gpio_write(h, LEFT_DIR, dir_val)
    lgpio.tx_pwm(h, LEFT_PWM, PWM_FREQ, abs(speed_percent), 0, 0)
    print(f"  [LEFT] Speed={speed_percent}% DIR={dir_val} ({'FORWARD' if forward else 'REVERSE'})")


def set_right(h, speed_percent, forward=True):
    dir_val = 1 if forward else 0
    lgpio.gpio_write(h, RIGHT_DIR, dir_val)
    lgpio.tx_pwm(h, RIGHT_PWM, PWM_FREQ, abs(speed_percent), 0, 0)
    print(f"  [RIGHT] Speed={speed_percent}% DIR={dir_val} ({'FORWARD' if forward else 'REVERSE'})")


def stop_all(h):
    lgpio.gpio_write(h, LEFT_DIR, 0)
    lgpio.gpio_write(h, RIGHT_DIR, 0)
    lgpio.tx_pwm(h, LEFT_PWM, PWM_FREQ, 0, 0, 0)
    lgpio.tx_pwm(h, RIGHT_PWM, PWM_FREQ, 0, 0, 0)
    print("  [ALL] STOPPED")


def main():
    print("=" * 60)
    print(" 🏎️  SMARTELEX 15D DIRECT MOTOR DIAGNOSTIC TEST")
    print("=" * 60)
    
    h = open_gpio()
    try:
        setup_pins(h)
        stop_all(h)
        time.sleep(1)

        print("\n--- TEST 1: LEFT MOTOR FORWARD (25% power) ---")
        set_left(h, 25, forward=True)
        time.sleep(2)

        print("\n--- TEST 2: LEFT MOTOR REVERSE (25% power) ---")
        set_left(h, 25, forward=False)
        time.sleep(2)

        set_left(h, 0)
        time.sleep(1)

        print("\n--- TEST 3: RIGHT MOTOR FORWARD (25% power) ---")
        set_right(h, 25, forward=True)
        time.sleep(2)

        print("\n--- TEST 4: RIGHT MOTOR REVERSE (25% power) ---")
        set_right(h, 25, forward=False)
        time.sleep(2)

        set_right(h, 0)
        time.sleep(1)

        print("\n--- TEST 5: BOTH MOTORS FORWARD TOGETHER (30% power) ---")
        set_left(h, 30, forward=True)
        set_right(h, 30, forward=True)
        time.sleep(3)

        print("\n--- TEST 6: STOPPING ALL MOTORS ---")
        stop_all(h)

        print("\n✅ MOTOR TEST COMPLETE!")

    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping motors...")
        stop_all(h)
    finally:
        lgpio.gpiochip_close(h)
        print("GPIO chip closed.")


if __name__ == "__main__":
    main()
