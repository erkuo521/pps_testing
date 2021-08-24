import os
import time 
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error import RPi.GPIO!")

interrupt_pin = 27
GPIO.setmode(GPIO.BCM) 
GPIO.setup(interrupt_pin,GPIO.IN) # channel used as IMU data ready detection
time.sleep(0.4)
GPIO.add_event_detect(interrupt_pin,GPIO.RISING)   

timestrt_date, temp_strt = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime()), time.time() #get date and start ms
f = open("data/pps_data" + timestrt_date + '.txt', "w")
f.write('start time={0} temp strt={1}\n'.format(timestrt_date,temp_strt)) 
print('start time={0} temp strt={1}\n'.format(timestrt_date,temp_strt))

last_time = temp_strt
error_list = []

while time.time() - temp_strt < 3600:
    if GPIO.event_detected(interrupt_pin):
        cur_time = time.time() 
        delta, run_time = cur_time - last_time, cur_time - temp_strt
        if (delta>2 or delta<0.8) and run_time > 10:
            error_list.append(cur_time)
        log_str = 'pps time={0} intverval={1} running time={2}s, error list={3}\n'.format(
                    cur_time,delta, run_time, error_list)
        f.write(log_str) 
        f.flush() # write to internal buffer, when full it will write to file by call()
        os.fsync(f) # force write to file now
        last_time = cur_time
        print(log_str)

    
f.close()


