import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)) +'/../lib')
from numpy import sqrt, cos, arctan2, sin, pi
from multiprocessing import Process
import json
from pymavlink import mavutil
from trajectory import Trajectory
from libtol import execute_SNOPT
from collections import namedtuple
import pdb
import select

class Mission:
	def __init__(self, ka_location, ac_address):

		#assign mavlink address
		self.ac = namedtuple("aircraft","name address lat lon alt")
		self.ac.name = 'tempest'
		self.ac.address = ac_address

		#assign Ka-1 Radar Datum
		self.ka = namedtuple("Ka1","lat lon alt")
		self.ka.lat = ka_location[0]
		self.ka.lon = ka_location[1]
		self.ka.alt = ka_location[2]

		#Connect to the aircraft
		self.connected = False
		self.connectAC()

		self.legcount = 0

		self.results = {}

		self.oldwpcount = 0

		self.trajectory = Trajectory(self.ka, self.master)

	def setAC(self,ac):
		'''Allows user to set another aircraft'''
		self.ac = ac.lower()

	def empty_socket(self):
		"""remove the data present on the socket"""
		input = [self.master.port]
		while 1:
			inputready, o, e = select.select(input,[],[], 0.0)
			if len(inputready)==0: break
			for s in inputready: s.recv(1)

	def connectAC(self):
		'''Connects to aircraft'''
		self.master = mavutil.mavlink_connection(self.ac.address)

		sys.stdout.flush()
		timeout = 5 #seconds
		t0 =  time.time()
		msg = None
		elapsed = 0
		while time.time() - t0 <= timeout and msg == None:
			msg = self.master.wait_heartbeat(blocking=False)
			if time.time()-t0 >= elapsed:
				print('Waiting for aircraft heartbeat...'),
				print (str(elapsed)+'\r'),
				sys.stdout.flush()
				elapsed += 1

		if msg == None:
			#raise StandardError('Cannot communicate with the aircraft.')
			print ('Waiting for aircraft heartbeat...failure')
			self.connect_status = 'Not connected'
		else:
			print 'Waiting for aircraft heartbeat...success'
			self.connected = True
			self.connect_status = 'Connected'
		time.sleep(1)

	def locateAC(self):

		if self.legcount == 0:
			'''Grabs a GPS message'''
			print('Waiting for GPS message...')
			sys.stdout.flush()
			self.empty_socket()
			msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

			#assign vehicle location
			self.ac.lat = msg.lat/1e7
			self.ac.lon = msg.lon/1e7
			self.ac.alt = msg.alt/1e3

			#Convert degrees to radians:
			ka_lat_rad = self.ka.lat * pi/180
			ka_lon_rad = self.ka.lon * pi/180

			ac_lat_rad = self.ac.lat * pi/180
			ac_lon_rad = self.ac.lon * pi/180

			dlat = ac_lat_rad - ka_lat_rad
			dlon = ac_lon_rad - ka_lon_rad

			#Haversine formula:
			R = 6371000
			a = sin(dlat/2)*sin(dlat/2) + cos(ka_lat_rad)*cos(ac_lat_rad)*sin(dlon/2.0)*sin(dlon/2.0)
			c = 2.0 * arctan2( sqrt(a), sqrt(1-a) )
			d = R * c	#distance
			b = arctan2( sin(dlon)*cos(ac_lat_rad), cos(ka_lat_rad)*sin(ac_lat_rad) - sin(ka_lat_rad)*cos(ac_lat_rad)*cos(dlon) )

			self.EastFromDatum = d*cos(pi/2-b)	#//x meters offset from Datum (ENU)
			self.NorthFromDatum = d*sin(pi/2-b)	#//y meters offset from Datum (ENU)
			self.UpFromDatum = self.ac.alt - self.ka.alt  #//z meters offset from Datum (ENU), should never be negative lol
			self.start = (self.EastFromDatum, self.NorthFromDatum, self.UpFromDatum, 0, 0, 0, 0, 0, 0, 0, 0) #assume unknown except GPS location
		
		else:
			print('Rseuming from last leg')
			self.start = (self.trajectory.east[-1], self.trajectory.north[-1], self.trajectory.up[-1], 0, 0, 0, 0, 0, 0, 0, 0) #assume unknown except GPS location


		self.int_start = list(self.start) #set 
		#self.int_goal = list(self.goal)

	def intermediate_goal(self):
		'''Sets an intermediate goal if final goal is further than 1 leg length'''
		#default leg length
		leg_length = 150

		#calculate XY distance from current position to final goal
		dist2goal = sqrt((self.goal.east - self.int_start[0])**2 + (self.goal.north - self.int_start[1])**2)

		#radius

		if dist2goal > self.goal.radius: #formerly leg_length
			chi = arctan2(self.goal.north-self.int_start[1],self.goal.east-self.int_start[0])
			self.int_goal = [round(dist2goal*cos(chi),1), round(dist2goal*sin(chi),1), round(0,1), 0]
		else:
			self.int_goal = [0, -100, 0, self.goal.radius]
			self.problemtype = "S10"
			self.mission_incomplete = False


	def check_with_user(self):
		'''Provides a summary of the impending SNOPT run to the user console'''
		print "~~~~~~~~~~~~~~~~~~~~~~~~~~~ Leg #: %i (%.1f%% complete) ~~~~~~~~~~~~~~~~~~~~~~~~~~" % (self.legcount,self.percent_complete)
		print 'Positions relative to Ka-1 Radar location'
		print "Simulated Position: {0:.2f} East (m), {1:.2f} North (m), {2:.2f} Up (m)".format(self.int_start[0], self.int_start[1], self.int_start[2])
		print "Final Goal:         {0:.2f} East (m), {1:.2f} North (m), {2:.2f} Up (m)".format(self.goal.east,self.goal.north,self.goal.up)

		if self.int_start[3] != 0:
			print "Next trajectory initial state:"
			print "Va:   %.4f" % self.int_start[3]
			print "gam:  %.4f" % self.int_start[4]
			print "chi:  %.4f" % self.int_start[5]
			print "phi:  %.4f" % self.int_start[6]
			print "dphi: %.4f" % self.int_start[7]
			print "CL:   %.4f" % self.int_start[8]
			print "T:    %.4f" % self.int_start[9]
		print "Aircraft:          %s" % self.ac.name
		print "Problem:           %s" % self.problemtype


	def clean_SNOPT(self):
		'''cleans up SNOPT leftovers'''
		try:
			os.remove('GeneralWind.out')
		except OSError:
			pass

	def launch_SNOPT(self):
		'''execute SNOPT (C++ Function from libflightplan.so)'''
		#suppresses SNOPT output if True
		quiet = True

		p1 = Process(target=execute_SNOPT,args=((self.int_start),(self.int_goal),self.ac.name,self.problemtype,))

		#p1 = Process(target=execute_SNOPT,args=((self.int_start),(300, 0, 0, 0),self.ac.name,self.problemtype,))

		#temporarily redirect stdout
		if quiet:
			devnull = open('/dev/null', 'w')
			oldstdout_fno = os.dup(sys.stdout.fileno())
			os.dup2(devnull.fileno(), 1)
		p1.start()
		if quiet:
			os.dup2(oldstdout_fno, 1)

		#t1 = threading.Thread(target=eng.plotSNOPT.plotnow, args=(os.getcwd(),), kwargs=dict(nargout=0)) 
		#t1.start()

		#display dots instead of SNOPT is quiet
		if quiet:
			dots = 0
			while p1.is_alive():
				running_str = ("Running SNOPT" + "." * dots + str(dots) + 's')
				# \r prints a carriage return first, so `b` is printed on top of the previous line.
				sys.stdout.write(running_str+'\r')
				sys.stdout.flush()
				dots = dots + 1 
				time.sleep(1.0)

		#waits for process to full complete before continuing
		p1.join()
		#t1.join()

	def stitch_trajectory(self):
		'''Stitches sequential trajectories together'''

		#open snopt_results json file generated by SNOPT run
		with open('snopt_results.json') as data_file:   
			most_recent_run = json.load(data_file)

		#store leg
		self.results['leg'+str(self.legcount)] = most_recent_run

		#stitch newest trajectory to end of previous ones	
		self.trajectory.t += (self.start_time + t*most_recent_run["dt"] for t in range(0,101))
		self.trajectory.east += (y + self.int_start[0] for y in most_recent_run["trajectory"]["y"])
		self.trajectory.north += (x + self.int_start[1] for x in most_recent_run["trajectory"]["x"])
		self.trajectory.up += (-z + self.int_start[2] for z in most_recent_run["trajectory"]["z"])
		self.trajectory.Va += (Va for Va in most_recent_run["trajectory"]["Va"])
		self.trajectory.gam += (gam for gam in most_recent_run["trajectory"]["gam"])
		self.trajectory.chi += (chi for chi in most_recent_run["trajectory"]["chi"])
		self.trajectory.phi += (phi for phi in most_recent_run["trajectory"]["phi"])
		self.trajectory.CL += (CL for CL in most_recent_run["trajectory"]["CL"])
		self.trajectory.dphi += (dphi for dphi in most_recent_run["trajectory"]["dphi"])
		self.trajectory.dCL += (dCL for dCL in most_recent_run["trajectory"]["dCL"])
		self.trajectory.T += (T for T in most_recent_run["trajectory"]["T"])

		#advance AC to end of last trajectory
		self.start_time = self.trajectory.t[-1]
		self.int_start[0] = round(self.trajectory.east[-1],4)
		self.int_start[1] = round(self.trajectory.north[-1],4)
		self.int_start[2] = round(self.trajectory.up[-1],4)
		self.int_start[3] = round(self.trajectory.Va[-1],4)
		self.int_start[4] = round(self.trajectory.gam[-1],4)
		self.int_start[5] = round(self.trajectory.chi[-1],4)
		self.int_start[6] = round(self.trajectory.phi[-1],4)
		self.int_start[7] = round(self.trajectory.CL[-1],4)
		self.int_start[8] = round(self.trajectory.dphi[-1],4)
		self.int_start[9] = round(self.trajectory.dCL[-1],4)
		self.int_start[10] = round(self.trajectory.T[-1],4)

	def check_completion(self):
		self.percent_complete = 100*sqrt((self.int_start[0] - self.start[0])**2+(self.int_start[1] - self.start[1])**2+(self.int_start[2] - self.start[2])**2)/sqrt((self.goal.east-self.start[0])**2 + (self.goal.north-self.start[1])**2 + (self.goal.up-self.start[2])**2)
		if self.percent_complete > 95:
			if self.goal.radius is 0: #if no loiter radius was set, end mission
				self.mission_incomplete = False
			else:				#otherwise run a loiter mission
				self.loiter = True

	def major_leg_summary(self):
		print "\nMajor Leg #1 finished. ({0} type)".format(self.problemtype)
		print "Completed in {0} minor legs".format(self.legcount)
		print "Total optimization time: {0:.2f} seconds".format(self.tf-self.t0)
		print "Total trajectory time: {0:.2f} seconds".format(self.trajectory.t[-1])

	def plot_stitched(self):
		#start MATLAB engine (for plotting)
		import matlab.engine
		self.eng = matlab.engine.start_matlab()
		self.eng.addpath('../matlab','../matlab/jsonlab')
		self.eng.figure()
		matlabx = matlab.double(self.trajectory.x)
		matlaby = matlab.double(self.trajectory.y)
		matlabz = matlab.double(self.trajectory.z)
		self.eng.plot3(matlabx,matlaby,matlabz, nargout = 0)
		self.eng.axis('equal', nargout = 0)
		self.eng.grid('on', nargout = 0)

	def run(self, goal):
		'''Runs the SNOPT loops'''
		self.percent_complete = 0.0
		self.start_time = 0
		self.start = [0]*11
		#mission starts as 'incomplete'
		self.mission_incomplete = True
		self.loiter = False
		#default to Guidance (no thrust)
		self.problemtype = 'G7'

		#assign goal
		self.goal = namedtuple("goal","x y z r")
		self.goal.east = goal[0]
		self.goal.north = goal[1]
		self.goal.up = goal[2]
		self.goal.radius = goal[3]

		#locate aircraft
		self.locateAC()

		#initialize empty stitched trajectory
		#self.trajectory = Trajectory(self.ka, self.master)



		self.t0 = time.time()



		while self.mission_incomplete:
			self.legcount += 1
			self.intermediate_goal()
			self.check_with_user()
			self.clean_SNOPT()
			self.launch_SNOPT()
			self.stitch_trajectory()
			self.check_completion()
			
		self.tf = time.time()

		self.major_leg_summary()
		#self._plot_stitched()

	def write_to_json(self,filename):
		with open(filename, 'w') as outfile:
			json.dump(self.results,outfile)