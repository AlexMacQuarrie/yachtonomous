from machine import Pin, I2C
import time

# ICM-20948 I2C Address (default AD0 = HIGH)
ICM20948_ADDR = 0x69  # Try 0x68 if needed

# Important Register Addresses
WHO_AM_I = 0x00       # WHO_AM_I register (should return 0xEA)
PWR_MGMT_1 = 0x06     # Power management register

# Initialize I2C1 on GP2 (SDA) and GP3 (SCL)
i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=100000)

# 1. Scan for devices
print("Scanning I2C bus...")
devices = i2c.scan()

print(devices)
if not devices:
    print("No I2C devices found")
else:
    print(f"Found device(s): {[hex(d) for d in devices]}")
    


'''
# 2. Check WHO_AM_I register
try:
    who_am_i = i2c.readfrom_mem(ICM20948_ADDR, WHO_AM_I, 1)
    print(f"WHO_AM_I response: {hex(who_am_i[0])}")

    if who_am_i[0] == 0xEA:
        print("ICM-20948 detected and verified.")
    else:
        print(f"Unexpected WHO_AM_I response: {hex(who_am_i[0])}")

except Exception as e:
    print(f"Failed to read WHO_AM_I: {e}")

# 3. Reset the ICM-20948
try:
    i2c.writeto_mem(ICM20948_ADDR, PWR_MGMT_1, b'\x80')  # Set reset bit
    print("Sent reset command to ICM-20948. Waiting...")
    time.sleep(0.1)  # Small delay for reset to complete

    # Read WHO_AM_I again after reset
    who_am_i = i2c.readfrom_mem(ICM20948_ADDR, WHO_AM_I, 1)
    print(f"After reset, WHO_AM_I response: {hex(who_am_i[0])}")

except Exception as e:
    print(f"Error sending reset command: {e}")

# 4. Wake up the sensor from sleep mode
try:
    i2c.writeto_mem(ICM20948_ADDR, PWR_MGMT_1, b'\x01')  # Clear sleep bit
    print("Woke up ICM-20948 from sleep mode.")
except Exception as e:
    print(f"Error waking up sensor: {e}")
'''

