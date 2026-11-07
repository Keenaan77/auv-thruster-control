from pymavlink import mavutil

master = mavutil.mavlink_connection('COM11', baud=115200)  # ganti COM5
print("Nunggu heartbeat...")
master.wait_heartbeat()
print("Nyambung!")