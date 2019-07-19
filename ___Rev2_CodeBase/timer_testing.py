
import datetime



timer_1hz = datetime.datetime.now()
timer_10hz = datetime.datetime.now()
Hz_1 = False
Hz_10 = False
seconds = 0
last_seconds = 0
BB_LED1 = False
rec_blink_count = 0
while True:
    now = datetime.datetime.now()
    # ------------------ 1 Hz timer ----------------------
    seconds = (now - timer_1hz).microseconds/1000000
    if abs(last_seconds - seconds) >= 0.300 : # detects the roll over edge in the microseconds timer
        timer_1hz = now # resets the timer
        Hz_1 = True # only on for a single pass once a second
    else: Hz_1 = False
    last_seconds = seconds
    # ------ End of ---- 1 Hz timer ----------------------
    # ------------------ 10 Hz timer ----------------------
    if abs(now - timer_10hz).microseconds/1000000 >= 0.100 :
        timer_10hz = now # resets the timer
        Hz_10 = True # only on for a single pass once a second
    else:
        Hz_10 = False
    # ------ End of ---- 10 Hz timer ----------------------

    if Hz_1:
        rec_blink_count = 0
    if Hz_10:
        rec_blink_count += 1
        if rec_blink_count == 1 or rec_blink_count == 3 or rec_blink_count == 4 or rec_blink_count == 6:
            BB_LED1 = not BB_LED1        
    if BB_LED1: print("ON")
    else: print("OFF")
