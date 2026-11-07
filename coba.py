from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('COM11', baud=115200)
master.wait_heartbeat()
print("Nyambung!")

# 1550 us = putaran pelan ke satu arah (1500 = berhenti)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
    1, mavutil.mavlink.MOTOR_TEST_THROTTLE_PWM, 1550, 2, 0, 0, 0)

# tampilkan balasan dari Pixhawk (diterima / ditolak)
t0 = time.time()
while time.time() - t0 < 3:
    m = master.recv_match(blocking=True, timeout=1)
    if m and m.get_type() in ("COMMAND_ACK", "STATUSTEXT"):
        print(m)
print("Kelar.")