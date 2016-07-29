# constants
uint32 LSD_IDLE = 0           # PTAM not running.
uint32 LSD_LOST = 1           # ptam is running, but lost
uint32 LSD_GOOD = 2           # tracking quality OK
uint32 LSD_TOOKKF = 3         # just took a new KF (equivalent to PTAM_BEST)

# header
Header      header

# ----------------- raw 10d filter state ----------------------------
float32     x
float32     y
float32     z
float32     dx
float32     dy
float32     dz
float32     roll
float32     pitch
float32     yaw
float32     dyaw



# --------------------- other values ---------------------------
float32     scale         # ptam scale factor (PTAMpos * scale = WORLDpos).
uint32      lsdState
float32     scaleAccuracy # if scale is very inaccurate, this is about 0.5, and grows up to 1 (=very accurate).


# ----------------- propagated from drone messages: -----------------
# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
uint32       droneState
float32      batteryPercent    # 0 means no battery, 100 means full battery
