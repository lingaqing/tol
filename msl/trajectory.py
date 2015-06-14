from fastkml import kml, styles
from fastkml.geometry import Geometry, Point, LineString, Polygon
from numpy import sqrt, cos, arctan2, sin, pi
from pymavlink import mavutil, mavwp
from collections import namedtuple
import json
import pdb
import time 

class Trajectory:
	def __init__(self, ka, master):
		self.master = master	
		self.ka = ka
		self.t = []
		self.east = []
		self.north = []
		self.up = []
		self.Va = []
		self.gam = []
		self.chi = []
		self.phi = []
		self.CL = []
		self.dphi = []
		self.dCL = []
		self.T = []
		self.lastindex = 0

	def send_to_ac(self, master):
		#self.master = mavutil.mavlink_connection(self.ac_address)
		#print "waiting for heartbeat"
		#pdb.set_trace()
		#self.master.wait_heartbeat(blocking=True)
		#print "received heartbeat"

		wp = mavwp.MAVWPLoader()
		frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

		#build waypoints
		coords = []
		seq = 0

		# wp.add(mavutil.mavlink.MAVLink_mission_item_message(
		# 	master.target_system,
		# 	master.target_component,
		# 	seq,
		# 	frame,
		# 	mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
		# 	0, 
		# 	0, 
		# 	0, 
		# 	16, #m/s 
		# 	0, 
		# 	0,
		# 	0, 
		# 	0, 
		# 	0))

		# seq += 1

		# #add 300m line in front soaring trajectory
		# for x in range(1,301,50):
		# 	coord = [0]*3
		# 	coord[1] = self.ka.lat
		# 	coord[0] = self.ka.lon - x/(111111.0*cos(coord[1])) #lon
		# 	coord[2] = 70#alt
		# 	coords.append(tuple(coord))
		# 	wp.add(mavutil.mavlink.MAVLink_mission_item_message(
		# 				self.master.target_system,
		# 				self.master.target_component,
		# 				seq,
		# 				frame,
		# 				mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
		# 				0, 0, 0, 10, 0, 0,
		# 				coord[1], coord[0], coord[2]))
		# 	seq += 1

		#add trajectory as waypoints
		for i in range(0,len(self.north)-self.lastindex):
			if i % 20 == 0:
				coord = [0]*3
				coord[1] = self.ka.lat + self.north[i+self.lastindex]/111111.0 #lat
				coord[0] = self.ka.lon + (self.east[i+self.lastindex])/(111111.0*cos(coord[1]*pi/180)) #lon
				coord[2] = self.up[i+self.lastindex] #relative alt
				coords.append(tuple(coord))
				wp.add(mavutil.mavlink.MAVLink_mission_item_message(
										master.target_system,
										master.target_component,
										seq,
										frame,
										mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
										0,  #current
										0,  #autocontinue
										0,
										30,
										0, #orbit (m) 
										0, #yaw orientation (NAV and LOITER missions)
										coord[1], #lat
										coord[0], #lon
										coord[2])) #alt

				seq += 1

				wp.add(mavutil.mavlink.MAVLink_mission_item_message(
							master.target_system,
							master.target_component,
							seq,
							frame,
							mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
							0, 
							0, 
							0, 
							self.Va[i+self.lastindex], 
							0, 
							0,
							0, 
							0, 
							0))
				seq += 1


		wp.save('waypoints.txt')
		print 'waypoint count: {0}'.format(wp.count())
		master.waypoint_clear_all_send()
		master.waypoint_count_send(wp.count())

		waypoint_id = 0
		while waypoint_id < wp.count()-1:
			msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
			time.sleep(.5) #artifical delay for SITL, REMOVE IN FIELD!!!
			print msg
			waypoint_id = msg.seq
			master.mav.send(wp.wp(msg.seq))
			print 'Sending waypoint {0}'.format(msg.seq)

		#Set active waypoint to 1
		master.waypoint_set_current_send(1)
		msg = master.recv_match(type=['MISSION_CURRENT'],blocking=True)
		print msg
		self.lastindex = len(self.north)-1
		return wp.count()

	def write_to_json(self,filename):
		jsonobj = {'t': self.t, 'east': self.east, 'north': self.north,\
		'up':self.up, 'Va': self.Va, 'gam': self.gam, 'chi': self.chi, 'phi':self.phi,\
		'CL':self.CL, 'dphi': self.dphi, 'dCL': self.dCL, 'T': self.T}
		with open(filename, 'w') as outfile:
			json.dump(jsonobj,outfile)

	def read_from_json(self,filename):
		with open(filename) as data_file:   
			loaded_trajectory = json.load(data_file)

		self.t = loaded_trajectory['t']
		self.east = loaded_trajectory['east']
		self.north = loaded_trajectory['north']
		self.up = loaded_trajectory['up']
		self.Va = loaded_trajectory['Va']
		self.gam = loaded_trajectory['gam']
		self.chi = loaded_trajectory['chi']
		self.phi = loaded_trajectory['phi']
		self.CL = loaded_trajectory['CL']
		self.dphi = loaded_trajectory['dphi']
		self.dCL = loaded_trajectory['dCL']
		self.T = loaded_trajectory['T']

	def write_to_kml(self,filename):
		k = kml.KML()
		ns = '{http://www.opengis.net/kml/2.2}'
		d = kml.Document(ns=ns, name='SNOPT Stitched Trajectory')

		#styles
		s = styles.Style(id='yellowLineGreenPoly')
		ls = styles.LineStyle(color='7f00ff00',width=4)
		ps = styles.PolyStyle(color='7f00ff00')
		s.append_style(ls)
		s.append_style(ps)
		d.append_style(s)

		#placemark
		pm = kml.Placemark(name='Stitched Trajectory',description='Trajectory for EADDDAS',styleUrl='#yellowLineGreenPoly')
		geom = Geometry()
		coords = []
		for i,x in enumerate(self.east):
			coord = [0]*3
			coord[1] = self.ka.lat + self.north[i]/111111.0 #lat
			coord[0] = self.ka.lon + self.east[i]/(111111.0*cos(coord[1]*pi/180)) #lon
			coord[2] = self.ka.alt + self.up[i] #alt
			coords.append(tuple(coord))
		geom.geometry = LineString(coords)
		geom.extrude = True
		geom.tessellate = True
		geom.altitude_mode = 'absolute'
		pm.geometry = geom
		d.append(pm)
		k.append(d)
		kmlfile = open(filename,"w")
		kmlfile.write(k.to_string(prettyprint=True))
		kmlfile.close()	