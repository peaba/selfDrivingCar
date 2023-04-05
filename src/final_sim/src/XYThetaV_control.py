#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

### parameter###
K = [[-1.0212,0.0,0.0],
     [0.0,-0.6212,-1.1733]];
### parameter###

dt = 0.1
maxv = 0.5
maxw = 1

way_point_number = 20
waypointdata = []

carpose = Twist()
command = Twist()

Start_flag = 0
waypoint_in = False

def limit(speed, max_value, min_value):
	if speed > max_value:
		speed = max_value
	if speed < min_value:
		speed = min_value
	return speed

def changedegree(degree):
	while degree > math.pi or degree <= -math.pi:
		if degree > math.pi:
			degree -= 2*math.pi
		elif degree <= math.pi:
			degree += 2*math.pi
	return degree

def generate_path(sx,sy,sth,gx,gy,gth):
    ki = math.sqrt(pow(gx-sx,2)+pow(gy-sy,2))*3/10
    kf = math.sqrt(pow(gx-sx,2)+pow(gy-sy,2))*3/10
    if math.cos(sth)*(gx-sx) + math.sin(sth)*(gy-sy) < 0:
        ki *= -1
        kf *= -1
    T = 100 # divide
    total_length = 0 # total_length = average_velocity 

    ax = ki * math.cos(sth) + sx
    ay = ki * math.sin(sth) + sy
    bx = -kf * math.cos(gth) + gx
    by = -kf * math.sin(gth) + gy
    xm = ( ax + bx ) / 2
    ym = ( ay + by ) / 2

    x = sx # xd
    y = sy # yd 
    th = sth
    s = 0

    for i in range(T):
        s_ = s
        s = (i + 1) / 100.0 # T =  100.0
        x_ = x #old x
        y_ = y #old y
        x = pow(1-s,4)*sx + 4*s*pow(1-s,3)*ax + 6*pow(s,2)*pow(1-s,2)*xm + 4*pow(s,3)*(1-s)*bx + pow(s,4)*gx
        y = pow(1-s,4)*sy + 4*s*pow(1-s,3)*ay + 6*pow(s,2)*pow(1-s,2)*ym + 4*pow(s,3)*(1-s)*by + pow(s,4)*gy
        th = math.atan2((y-y_)/(s-s_),(x-x_)/(s-s_))
        if  math.cos(sth)*(gx-sx) + math.sin(sth)*(gy-sy) < 0:
            th = changedegree(th + math.pi)
        total_length += math.sqrt(pow((x-x_),2)+pow((y-y_),2))  

    return round(total_length,3), round(xm,3), round(ym,3)

#def pose_callback(data):#get robot positions
#	carpose.linear.x = data.linear.x
#	carpose.linear.y = data.linear.y
#	carpose.angular.z = changedegree(data.angular.z)
#	global Start_flag
#	if Start_flag == 0:
#		Start_flag = 1

def pose_callback(msg):#get robot positions
	carpose.linear.x = msg.data[0]
	carpose.linear.y = msg.data[1]
	carpose.angular.z = changedegree(msg.data[2])
	global Start_flag
	if Start_flag == 0:
		Start_flag = 1

def waypoint_callback(msg):
    global waypoint_in
    if waypoint_in == False:
        global way_point_number,waypointdata
        waypointdata = []
        way_point_number = len(msg.data)/4
        for i in range(way_point_number):
    	       waypointdata += [[msg.data[4*i],msg.data[4*i+1],msg.data[4*i+2],msg.data[4*i+3]]]
        waypoint_in = True
        
def Tracking():
    rospy.init_node('lab3_pt', anonymous=True)
#    rospy.Subscriber("/robot_pose", Twist, pose_callback)
    rospy.Subscriber("/status", Float64MultiArray, pose_callback)
    rospy.Subscriber('/way_point', Float64MultiArray, waypoint_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

    rate = rospy.Rate(10) # 10hz

    max_w = 0
    min_w = 0

    while not rospy.is_shutdown():
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        global Start_flag,waypoint_in


        if Start_flag == 1 and waypoint_in:
            
            px = carpose.linear.x
            py = carpose.linear.y
            pth = carpose.angular.z

            # start tracking
            m = 0 # m : current tracking path

            p = 0#p mean s', to get uniform distribution curve
            p_int = 0# integral
            pT = dt # 0 # pT is between
            s = 0
            ss = 0
            print(waypointdata)
            vi = waypointdata[m][3]
            vf = waypointdata[m+1][3]
            
            sx = px
            sy = py
            sth = pth
				 
            gx = waypointdata[m+1][0]
            gy = waypointdata[m+1][1]
            gth = waypointdata[m+1][2]

            xd = sx# first point
            yd = sy
            thd = sth

            # generate path
            # print(sx,sy,sth,gx,gy,gth)
            length, xm, ym = generate_path(sx,sy,sth,gx,gy,gth)

            T = 2 * length/(vi + vf)# in s scale # T = 2 * length / (vi + vf)   #(Trapezoid)
            vis = vi/length# in s scale #vis(in s) / 1 = vi(real) / length
            vfs = vf/length 

            ki = math.sqrt(pow(gx-sx,2)+pow(gy-sy,2))*3/10
            kf = math.sqrt(pow(gx-sx,2)+pow(gy-sy,2))*3/10

            if math.cos(sth)*(gx-sx) + math.sin(sth)*(gy-sy) < 0:
                ki *= -1
                kf *= -1

            k1 = K[0][0] # control gain calculate by matlab LMI
            k2 = K[1][1]
            k3 = K[1][2]
	
            if math.cos(sth)*(gx-sx) + math.sin(sth)*(gy-sy) < 0:
                k2 *= -1
                
            Start_flag = 2
            
        if Start_flag == 2:

            px = carpose.linear.x
            py = carpose.linear.y
            pth = carpose.angular.z
            # navigation part

            xd_ = xd#old xd
            yd_ = yd#old yd
            s = vis*pT-(vis-vfs)*pT*pT/(2*T)

            ax = ki * math.cos(sth) + sx
            ay = ki * math.sin(sth) + sy
            bx = -kf * math.cos(gth) + gx
            by = -kf * math.sin(gth) + gy
            # use p = s' to replace s
            ss_ = ss
            ss = s
            # p_ = p

            while p_int < s * length:
                ppx_ = pow(1-p,4)*sx + 4*p*pow(1-p,3)*ax + 6*pow(p,2)*pow(1-p,2)*xm + 4*pow(p,3)*(1-p)*bx + pow(p,4)*gx
                ppy_ = pow(1-p,4)*sy + 4*p*pow(1-p,3)*ay + 6*pow(p,2)*pow(1-p,2)*ym + 4*pow(p,3)*(1-p)*by + pow(p,4)*gy
                p += 0.0001
                ppx = pow(1-p,4)*sx + 4*p*pow(1-p,3)*ax + 6*pow(p,2)*pow(1-p,2)*xm + 4*pow(p,3)*(1-p)*bx + pow(p,4)*gx
                ppy = pow(1-p,4)*sy + 4*p*pow(1-p,3)*ay + 6*pow(p,2)*pow(1-p,2)*ym + 4*pow(p,3)*(1-p)*by + pow(p,4)*gy
                p_int += math.sqrt(pow(ppx-ppx_,2)+pow(ppy-ppy_,2))
            s = p
            # use p = s' to replace s 
            xd = pow(1-s,4)*sx + 4*s*pow(1-s,3)*ax + 6*pow(s,2)*pow(1-s,2)*xm + 4*pow(s,3)*(1-s)*bx + pow(s,4)*gx
            yd = pow(1-s,4)*sy + 4*s*pow(1-s,3)*ay + 6*pow(s,2)*pow(1-s,2)*ym + 4*pow(s,3)*(1-s)*by + pow(s,4)*gy

            if pT != 0:
                thd = round(math.atan2((yd-yd_)/(ss-ss_),(xd-xd_)/(ss-ss_)),5)
            if math.cos(sth)*(gx-sx) + math.sin(sth)*(gy-sy) < 0 and pT != 0:
             	thd = round(changedegree(thd+math.pi),5)
            s = ss
            dx=4*sx*pow(s-1,3) - 4*pow(s,3)*bx - 4*ax*pow(s-1,3) + 4*pow(s,3)*gx - 12*pow(s,2)*bx*(s-1) - 12*s*ax*pow(s-1,2) + 12*s*xm*pow(s-1,2) + 6*pow(s,2)*xm*(2*s-2)
            ddx=12*sx*pow(s-1,2) + 12*xm*pow(s-1,2) - 24*bx*pow(s,2) + 12*pow(s,2)*gx + 12*pow(s,2)*xm - 24*ax*pow(s-1,2) - 24*bx*s*(s-1) - 12*ax*s*(2*s-2) + 24*s*xm*(2*s-2)
            dy=4*sy*pow(s-1,3) - 4*pow(s,3)*by - 4*ay*pow(s-1,3) + 4*pow(s,3)*gy - 12*pow(s,2)*by*(s-1) - 12*s*ay*pow(s-1,2) + 12*s*ym*pow(s-1,2) + 6*pow(s,2)*ym*(2*s-2)
            ddy=12*sy*pow(s-1,2) + 12*ym*pow(s-1,2) - 24*by*pow(s,2) + 12*pow(s,2)*gy + 12*pow(s,2)*ym - 24*ay*pow(s-1,2) - 24*by*s*(s-1) - 12*ay*s*(2*s-2) + 24*s*ym*(2*s-2)
            s = p
		     
            vs = math.sqrt(pow(dx,2)+pow(dy,2)) # in s scale#
            ws = (ddy*dx-ddx*dy)/(pow(dx,2)+pow(dy,2))###*0.85/vs#
		     
            vs = length#

            if math.cos(sth)*(gx-sx) + math.sin(sth)*(gy-sy) < 0:
             	vs *= -1

            ds = vis-(vis-vfs)*pT/T
            vd = round(vs*ds,4) # in real scale
            wd = round(ws*ds,4)

            e1 = round(math.cos(pth)*(xd - px)+math.sin(pth)*(yd - py),5)
            e2 = round(-math.sin(pth)*(xd - px)+math.cos(pth)*(yd - py),5)
            e3 = round(changedegree(thd - pth),5)
		     
            u1 = k1 * e1
            u2 = k2 * e2 + k3 * e3

            v = vd * math.cos(e3) - u1
            w = wd - u2

            v = round(max(min(v,maxv),-maxv),2) # limit command
            w = round(max(min(w,maxw),-maxw),2)
            pT = pT + dt

            if pT >= T:
            	m += 1# tracking next path
            	if m < way_point_number-1:
            		p = 0#p mean s', to get uniform distribution curve
            		p_int = 0# integral
            		pT = 0 # 0 # pT is between
            		s = 0
            		ss = 0
                    
            		vi = waypointdata[m][3]
            		vf = waypointdata[m+1][3]
            
            		sx = px
            		sy = py
            		sth = pth
            		# prevent sudden location error cause whole path wrong
            		if abs(sx-waypointdata[m][0]) >= 0.5 or abs(sy-waypointdata[m][1]) >= 0.5 or changedegree(pth - waypointdata[m][2]) >= 0.5:
             			sx = waypointdata[m][0]
             			sy = waypointdata[m][1]
             			sth = waypointdata[m][2]
            		gx = waypointdata[m+1][0]
            		gy = waypointdata[m+1][1]
            		gth = waypointdata[m+1][2]

            		# prevent start point and end point too close
            		if math.sqrt(pow(gx-sx,2)+pow(gy-sy,2)) <= 0.1:
            			m += 1
            			gx = waypointdata[m+1][0]
            			gy = waypointdata[m+1][1]
            			gth = waypointdata[m+1][2]


            		# print(sx,sy,sth,gx,gy,gth)
            		length, xm, ym = generate_path(sx,sy,sth,gx,gy,gth)

            		T = 2 * length/(vi + vf)# in s scale # T = 2 * length / (vi + vf)   #(Trapezoid)
            		vis = vi/length# in s scale #vis(in s) / 1 = vi(real) / length
            		vfs = vf/length 

            		ki = math.sqrt(pow(gx-sx,2)+pow(gy-sy,2))*3/10
            		kf = math.sqrt(pow(gx-sx,2)+pow(gy-sy,2))*3/10
            		if math.cos(sth)*(gx-sx) + math.sin(sth)*(gy-sy) < 0:
             			ki *= -1
             			kf *= -1

            		k1 = K[0][0] # control gain calculate by matlab LMI
            		k2 = K[1][1]
            		k3 = K[1][2]
            		if math.cos(sth)*(gx-sx) + math.sin(sth)*(gy-sy) < 0:
             			k2 *= -1	

            if m >= way_point_number:
             	v = 0
             	w = 0
            command.linear.x = v
            command.angular.z = w
            print("v ",v," w ",w)

            if w > max_w:
                max_w = w
            if w < min_w:
                min_w = w

            pub.publish(command)
            if m >= way_point_number:
                print("max w = ",max_w," min w = ",min_w)
                print("tracking finish")
                m=0
                waypoint_in = False
                return 0
        rate.sleep()

if __name__ == '__main__':
    try:
        Tracking()
    except rospy.ROSInterruptException:
        pass
