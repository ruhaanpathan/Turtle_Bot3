#!/usr/bin/env python3
"""
Diagnose which gpiochip controls the Pi 5 header pins.
"""
import lgpio
import os
import time
import subprocess

print("=== GPIO Chip Detection ===\n")

# List all gpiochips
chips = sorted([f for f in os.listdir("/dev") if f.startswith("gpiochip")])
print(f"Available chips: {chips}\n")

# Get chip info
try:
    result = subprocess.run(["gpiodetect"], capture_output=True, text=True)
    if result.returncode == 0:
        print("gpiodetect output:")
        print(result.stdout)
except FileNotFoundError:
    print("(gpiodetect not installed — install with: sudo apt install gpiod)\n")

# Try each chip — attempt to claim GPIO17 (encoder A) and read it
print("=== Testing each chip with GPIO17 ===\n")
for chip_name in chips:
    chip_num = int(chip_name.replace("gpiochip", ""))
    try:
        h = lgpio.gpiochip_open(chip_num)
        info = lgpio.gpio_get_chip_info(h)
        print(f"gpiochip{chip_num}: opened (handle={h}), info={info}")
        
        # Try to claim GPIO17 as input and read it
        try:
            lgpio.gpio_claim_input(h, 17)
            val = lgpio.gpio_read(h, 17)
            print(f"  → GPIO17 claimed as input, reads: {val}")
            lgpio.gpio_free(h, 17)
            
            # Now try to toggle GPIO23 (IN1) as output and read back
            lgpio.gpio_claim_output(h, 23)
            lgpio.gpio_write(h, 23, 1)
            time.sleep(0.01)
            lgpio.gpio_free(h, 23)
            
            lgpio.gpio_claim_input(h, 23)
            val = lgpio.gpio_read(h, 23)
            print(f"  → GPIO23 toggled HIGH then read back: {val}")
            lgpio.gpio_free(h, 23)
            
            print(f"  ✅ gpiochip{chip_num} can access header pins!")
        except Exception as e:
            print(f"  ✗ Cannot access GPIO17/23: {e}")
        
        lgpio.gpiochip_close(h)
    except Exception as e:
        print(f"gpiochip{chip_num}: FAILED to open — {e}")
    print()

# Quick blink test on the working chip
print("=== Blink Test (GPIO23 = IN1 pin) ===")
print("If your multimeter is on GPIO23, you should see voltage toggling.\n")

for chip_name in chips:
    chip_num = int(chip_name.replace("gpiochip", ""))
    try:
        h = lgpio.gpiochip_open(chip_num)
        try:
            lgpio.gpio_claim_output(h, 23)
            for i in range(6):
                lgpio.gpio_write(h, 23, i % 2)
                state = "HIGH (3.3V)" if i % 2 else "LOW (0V)"
                print(f"  gpiochip{chip_num} GPIO23 = {state}")
                time.sleep(0.5)
            lgpio.gpio_free(h, 23)
            print(f"  ✅ If you saw voltage toggle, gpiochip{chip_num} is correct!\n")
        except Exception:
            pass
        lgpio.gpiochip_close(h)
    except Exception:
        pass
