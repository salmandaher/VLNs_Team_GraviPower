import rospy 
from vlns.srv import * 
from std_msgs.msg import String, Float32, Bool, Int32
import time

encoder = 0.0
mode = "charge" 
cars = 0
flag = 0
last_car=time.time()

last_encoder = 0.0
last_time = None
rpm = 0.0

PULSES_PER_REVOLUTION = 1440  # 360 degrees / 0.25 degrees per pulse

def publisher():
    global last, mode
    if mode != last:
        pub.publish(mode)
        rospy.loginfo(f"Changed mode to : {mode}")
        last = mode

def sges_server_callback(req):
    global mode
    mode = req.mode
    publisher()
    response = sgesResponse()
    response.success = True
    return response

def enc_cb(msg):
    global flag, encoder, mode, last_encoder, last_time, rpm, PULSES_PER_REVOLUTION
    current_time = time.time()
    encoder = msg.data

    if last_time is not None:
        dt = current_time - last_time
        if dt > 0:
            delta_pulses = encoder*1000 - last_encoder*1000
            pulses_per_sec = delta_pulses / dt
            rpm = (pulses_per_sec * 60)*60 / PULSES_PER_REVOLUTION
            
##      rospy.loginfo(f"Speed: {rpm:.2f} RPM")
    last_encoder = encoder
    last_time = current_time

    if encoder > 50.0:
        flag = 1
        mode = "maxCharge"
        publisher()
    else:
        flag = 0

def car_cb(msg):
    global cars, last_car
    if time.time() - last_car>1 and msg.data:
        last_car=time.time()
        cars+=1
        pubc.publish(cars)	
        
def light_cb(msg):
    global mode
    if msg.data > 1000.0 and (mode =='maxCharge' or mode =='charge'):
        mode = "discharge"
        publisher()
        
     
rospy.init_node('control_node')
rospy.Service('moder', sges, sges_server_callback)
pub = rospy.Publisher('mode', String, queue_size=10)
pubc = rospy.Publisher('cars_count', Int32, queue_size=10)
height_pub = rospy.Publisher('height', Float32, queue_size=10)
energy_pub = rospy.Publisher('energy', Float32, queue_size=10)
bts_pub = rospy.Publisher('bts', Int32, queue_size=10)

rospy.Subscriber('energy_storage/encoder', Float32, enc_cb)
rospy.Subscriber('/car_passed', Bool, car_cb)
rospy.Subscriber('lighting', Float32, light_cb)

last = None

height = Float32()
energy = Float32()

rate = rospy.Rate(10)  # 10 Hz update rate
rospy.loginfo('115050Server is running...')+150
while not rospy.is_shutdown():
    height.data = encoder * 105 / 32
    if height.data < 40.0:
        bts_pub.publish(255)
    if height.data > 40.0:
        bts_pub.publish(255)
    height_pub.publish(height)
    energy.data=height.data*0.01*11*9.81*0.000278
    energy_pub.publish(energy)
    rate.sleep()

rospy.spin()
