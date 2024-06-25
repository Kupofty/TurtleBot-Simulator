#!/usr/bin/env python

#############################################################################
# imports
#############################################################################
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from fsm import fsm
from kobuki_msgs.msg import BumperEvent

#############################################################################
# class RobotBehavior
#############################################################################

class RobotBehavior(object):
	#############################################################################
	# constructor, called at creation of instance
	#############################################################################
	def __init__(self, handle_pub, T):
		self.twist = Twist()
		self.twist_real = Twist()
		self.vreal = 0.0 # longitudial velocity
		self.wreal = 0.0 # angular velocity
		self.vmax = 1.5
		self.wmax = 4.0
		self.previous_signal = 0
		self.button_pressed = False
		self.joy_activated = False
		self.pub = handle_pub
		self.T = T
		
		#Compteurs de temps pour les transitions/actions
		self.cpt=0
		self.cpt1=0
		self.cpt2=0
		self.cpt4=0
		self.cpt5=0
	
		# instance of fsm with (état de départ, état d'arrivée, condition ,callback)
		self.fs = fsm([ ("Start","JoyControl", True ),
			    ("JoyControl","AutonomousMode1", self.check_JoyControl_To_AutonomousMode1, self.DoAutonomousMode1),
			    ("JoyControl","JoyControl", self.KeepJoyControl, self.DoJoyControl),
			    ("AutonomousMode1","JoyControl", self.check_AutonomousMode1_To_JoyControl, self.DoJoyControl),
			    ("AutonomousMode1","AutonomousMode1", self.KeepAutonomousMode1, self.DoAutonomousMode1),
			    ("AutonomousMode1","Wait_Stop", self.check_AutonomousMode1_To_Wait_Stop, self.DoWait_stop),
			    ("Wait_Stop","Wait_Stop", self.KeepWait_Stop , self.DoWait_stop),
			    ("Wait_Stop","Recule", self.check_Wait_Stop_To_Recule, self.DoRecule),
			    ("Recule","Recule", self.KeepRecule , self.DoRecule),
			    ("Recule","Wait_Stop_2",self.check_Recule_To_Wait_Stop_2 , self.DoWait_Stop_2),
			    ("Wait_Stop_2","Wait_Stop_2", self.KeepWait_Stop_2 , self.DoWait_Stop_2),
			    ("Wait_Stop_2", "Rotat" , self.check_Wait_Stop_2_To_Rotat , self.DoRotat),
			    ("Rotat","Rotat", self.KeepRotat , self.DoRotat),
			    ("Rotat","Wait_Stop_3", self.check_Rotat_To_Wait_Stop_3,self.DoWait_Stop_3),
			    ("Wait_Stop_3","Wait_Stop_3",self.KeepWait_Stop_3,self.DoWait_Stop_3),
			    ("Wait_Stop_3" , "AutonomousMode1" , self.check_Wait_Stop_3_To_AutonomousMode1, self.DoAutonomousMode1) ] )
		self.bumper_obstacle = False


	#############################################################################
	# callback for joystick feedback
	#############################################################################
	def callback(self,data):
		self.twist.linear.x = self.vmax * data.axes[1]
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = self.wmax*data.axes[2]

		# for transition conditions of fsm
		if (not self.button_pressed):
			self.button_pressed = (self.previous_signal==0 and data.buttons[0]==1)		
		self.previous_signal = data.buttons[0];
		self.joy_activated = (abs(data.axes[1])>0.001 or abs(data.axes[2])>0.001)

	def processBump(self, data):		
		if (data.state==data.PRESSED):
			self.bumper_obstacle=True;		#Fonction qui passe un booléen en vrai si le bumper est enfoncé (collision)
			self.bumper_nb=data.bumper
			rospy.loginfo("bumper PRESSED: no=%d", self.bumper_nb)

	def processScan(self,data):
		print(data.angle_max)		#PAS FINIS
		print(data.angle_min)		#Fonction qui permet de savoir si quelqu'un chose est détecté par le LIDAR, la distance, et a quel angle
		print(data.angle_increment )
		

	#############################################################################
	# smoothing velocity function to avoid brutal change of velocity
	#############################################################################
	def smooth_velocity(self):
		accmax = 0.01;
		accwmax = 0.05;
		vjoy = 0.0
		wjoy = 0.0
		vold = 0.0
		wold = 0.0	

		#filter twist
		vjoy = self.twist.linear.x
		vold = self.vreal
		deltav_max = accmax / self.T
	
		#vreal
		if abs(vjoy - self.vreal) < deltav_max:
			self.vreal = vjoy
		else:
			sign_ = 1.0
			if (vjoy < self.vreal):
				sign_ = -1.0
			else:
				sign_ = 1.0
			self.vreal = vold + sign_ * deltav_max
	
		#saturation
		if (self.vreal > self.vmax):
			self.vreal = self.vmax
		elif (self.vreal < -self.vmax):
			self.vreal = -self.vmax		
	
		#filter twist
		wjoy = self.twist.angular.z
		wold = self.wreal
		deltaw_max = accwmax / self.T
	
		#wreal
		if abs(wjoy - self.wreal) < deltaw_max:
			self.wreal = wjoy
		else:
			sign_ = 1.0
			if (wjoy < self.wreal):
				sign_ = -1.0
			else:
				sign_ = 1.0
			self.wreal = wold + sign_ * deltaw_max
			
		#saturation
		if (self.wreal > self.wmax):
			self.wreal = self.wmax
		elif (self.wreal < -self.wmax):
			self.wreal = -self.wmax		
			
		self.twist_real.linear.x = self.vreal	
		self.twist_real.angular.z = self.wreal
	
	####################################################################################
	# fonctions check : renvoie true ou false selon si la condition est vérifiée ou non
	####################################################################################
	def check_JoyControl_To_AutonomousMode1(self,fss):
		return self.button_pressed

	def check_AutonomousMode1_To_JoyControl(self,fss):
		return self.joy_activated

	def check_AutonomousMode1_To_Wait_Stop(self,fss):
		return self.bumper_obstacle

	def check_Wait_Stop_To_Recule (self, fss):
		return (self.cpt1>5)

	def check_Recule_To_Wait_Stop_2 (self, fss):
		return (self.cpt>10)
		
	def check_Wait_Stop_2_To_Rotat (self, fss):
		return (self.cpt2>5)
		
	def check_Rotat_To_Wait_Stop_3 (self, fss):
		return (self.cpt4 >8)

	def check_Wait_Stop_3_To_AutonomousMode1 (self, fss):
		return (self.cpt5>5)
		
	######################################################################################################
	# fonctions keep : continue de faire la même action selon si le check de transition est validé ou non
	######################################################################################################
	def KeepJoyControl(self,fss):
		return (not self.check_JoyControl_To_AutonomousMode1(fss))

	def KeepAutonomousMode1(self,fss):
		return (not (self.check_AutonomousMode1_To_JoyControl(fss) or self.check_AutonomousMode1_To_Wait_Stop(fss)))

	def KeepWait_Stop(self, fss):
		return (not self.check_Wait_Stop_To_Recule(fss) )

	def KeepWait_Stop_2(self,fss):
		return (not self.check_Wait_Stop_2_To_Rotat(fss))

	def KeepWait_Stop_3(self,fss):
		return (not self.check_Wait_Stop_3_To_AutonomousMode1(fss))

	def KeepRotat(self, fss):
	 	return (not self.check_Rotat_To_Wait_Stop_3(fss))

	def KeepRecule(self, fss):
		return (not self.check_Recule_To_Wait_Stop_2(fss))

	######################################################################################
	# fonctions do : indiquent la manoeuvre à effectuer et incrément le compteur de temps
	######################################################################################
	def DoJoyControl(self,fss,value):
		self.button_pressed =  False;
		self.smooth_velocity()
		self.pub.publish(self.twist_real)
		pass

	def DoAutonomousMode1(self,fss,value):
		self.button_pressed =  False;
		self.bumper_obstacle = False;
		go_fwd = Twist()
		go_fwd.linear.x = self.vmax/6.0
		self.pub.publish(go_fwd)
		self.cpt5=0
		pass

	def DoRecule (self, fss, value):
		Depl=Twist()
		Depl.linear.x= -self.vmax/6.0
		self.pub.publish(Depl)
		self.cpt1=0
		self.cpt =self.cpt+ 1
		
	def DoWait_stop(self,fss, value):
		arret=Twist()
		arret.linear.x= -0
		self.pub.publish(arret)
		self.cpt1= self.cpt1 +1

	def DoWait_Stop_2 (self, fss, value):
		arret2=Twist()
		arret2.linear.x= -0
		self.pub.publish(arret2)
		self.cpt=0
		self.cpt2 = self.cpt2 +1

	def DoWait_Stop_3 (self, fss, value):
		arret3=Twist()
		arret3.linear.x= -0
		self.pub.publish(arret3)
		self.cpt4=0
		self.cpt5 = self.cpt5 +1

	def DoRotat(self, fss, value):
		self.rot=Twist()
		self.rot.angular.z= -0.5*self.wmax
		self.pub.publish(self.rot)
		self.cpt2=0
		self.cpt4 = self.cpt4 + 1

#############################################################################
# main function
#############################################################################
if __name__ == '__main__':
	try:
		rospy.init_node('joy4ctrl')
		# real turtlebot2
		pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
		Hz = 10
		rate = rospy.Rate(Hz)
		T = 1.0/Hz

		MyRobot = RobotBehavior(pub,T);

		rospy.Subscriber("joy", Joy, MyRobot.callback)
		MyRobot.fs.start("Start")
		rospy.Subscriber("mobile_base/events/bumper", BumperEvent, MyRobot.processBump)
		rospy.Subscriber("scan", LaserScan, MyRobot.processScan)

		# loop at rate Hz
		while (not rospy.is_shutdown()):
			ret = MyRobot.fs.event("")
			rate.sleep()

	except rospy.ROSInterruptException:
        	pass
