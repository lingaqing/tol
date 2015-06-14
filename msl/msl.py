from mission import Mission
from ge_interface import ge_interface
import textwrap
import time
from multiprocessing import Process, Queue

from pymavlink import mavutil
import sys
import curses
import pdb
import os
import select

if __name__ == '__main__':
	print('Welcome to the TOL Mission Selection Logic')
	print('')
	print textwrap.dedent("""\
	%*=+--+=#=+--      EA-DDDAS Trajectory Optimization Layer         --+=#=+--+=#*%
	%          Copyright (C) 2015 Regents of the University of Colorado.           %
	%                             All Rights Reserved.                             %
	%                                                                              %
	%    This program is free software: you can redistribute it and/or modify      %
	%    it under the terms of the GNU General Public License version 2 as         %
	%    published by the Free Software Foundation.                                %
	%                                                                              %
	%    This program is distributed in the hope that it will be useful,           %
	%    but WITHOUT ANY WARRANTY; without even the implied warranty of            %
	%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             %
	%    GNU General Public License for more details.                              %
	%                                                                              %
	%    You should have received a copy of the GNU General Public License         %
	%    along with this program.  If not, see <http://www.gnu.org/licenses/>.     %
	%                                                                              %
	%           Will Silva                                                         %
	%           william.silva@colorado.edu                                         %
	%                                                                              %
	%*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*%
		""")

	raw_input('Press Enter to Continue')	

	#ALRIGHT, LET'S BEGIN

	#Default Ka-1 Radar Location (Table mountain)
	Ka1_location = [40.1451,-105.2408,1676]

	#Default connection to aircraft (SITL)
	ac_address = 'udp:localhost'
	io_port = '14550'
	gcs_port = '14551'

	#stuff for Auto Mode
	ottegoal = 1
	auto = False
	autostack = [2,1,2,1,2,1] #stack read from right to left because it's a pop stack
	oldwpcount = 0

	#Create mission instance (this will try to connect to the aircraft)
	mission = Mission(Ka1_location, ac_address + ':' + io_port)

	#Create ground control station thread (but don't start unless we are connected to aircraft)
	gcs = ge_interface(ac_address + ':' + gcs_port,Ka1_location)
	gcs.daemon = True

	if mission.connected:
		gcs.start()

	user_input = 0
	while True:
		clear = lambda : os.system('tput reset')
		clear()
		print("Ka-1 Location: {0}").format(Ka1_location)
		print("Aircraft address: {0} (IO: {1}, GCS: {2})").format(ac_address,io_port,gcs_port)
		if mission.connected:
			pass
			print("Aircraft Location (relative to Ka-1):")
			print("East (m): {0}").format(gcs.east)
			print("North (m): {0}").format(gcs.north)
			print("Up (m): {0}").format(gcs.up)
		else:
			print("NOT CONNECTED.")
		print("")
		print("Options:")
		print("1. Optimize to next available goal point")
		print("2. Send current trajectory")
		print("3. Set aircraft address")
		print("4. Set Ka-1 Location")
		print("5. Auto Mode")
		
		if auto ==  False:
			while sys.stdin in select.select([sys.stdin], [], [], 1)[0]:
				line = sys.stdin.readline()
				if line:
					user_input = line
				else: # an empty line means stdin has been closed
					print('eof')
					exit(0)
	
		if int(user_input) == 1:

			#Get goal from Michael Otte's code
			if ottegoal == 1:
				goal_from_otte = (400, 0, 70, 0) #East, North, Up, Radius, Relative to Ka1 Location
				ottegoal += 1
			elif ottegoal == 2:
				goal_from_otte = (400, 400, 70, 0) #East, North, Up, Radius, Relative to Ka1 Location
				ottegoal += 1
			elif ottegoal == 3:
				goal_from_otte = (800, 400, 70, 100) #East, North, Up, Radius, Relative to Ka1 Location

			#Run Mission
			mission.run(goal_from_otte)

			#Write final trajectory to files
			mission.write_to_json('trajectory.json')
			mission.trajectory.write_to_kml('trajectory.kml')
			mission.trajectory.write_to_json('trajectory_backup.json')
			user_input = 0

		if int(user_input) == 2:
			#Send stitched trajectory to aircraft
			if mission.connected:
				if mission.results:
					oldwpcount = mission.trajectory.send_to_ac(mission.master)
				else:
					print "Using backup file"
					goal_from_otte = (400, 0, 70, 100) #East, North, Up, Radius, Relative to Ka1 Location
					mission.trajectory.read_from_json('trajectory_backup.json')
					mission.trajectory.send_to_ac(mission.master)
					time.sleep(1)
			else:
				print('Not connected!')
			user_input = 0


		if int(user_input) == 3:
			#Set new addresses
			new_address = raw_input('New sUAS address: ')
			new_io_port = raw_input('New sUAS IO port: ')
			new_gcs_port = raw_input('New sUAS GCS port: ')
			if new_address == '':
				pass
			else:
				mission.ac.address = new_address + ':' + new_io_port
				mission.connectAC()
				if mission.connected:
					if gcs.is_alive():
						gcs.poison = True
					gcs = ge_interface(new_address + ':' + new_gcs_port,Ka1_location)
					gcs.daemon = True
					gcs.start()
					io_port = new_io_port
					gcs_port = new_gcs_port
			user_input = 0

		if int(user_input) == 4:
		#Set new addresses
			new_kalat = float(raw_input('New Ka-1 Latitude: '))
			new_kalon = float(raw_input('New Ka-1 Longitude: '))
			new_kaalt = float(raw_input('New Ka-1 Altitude: '))
			if new_kalat == '':
				pass
			else:
				Ka1_location = [new_kalat,new_kalon,new_kaalt]

				mission.ka.lat = new_kalat
				mission.ka.lon = new_kalon
				mission.ka.alt = new_kaalt

				gcs.kalat = new_kalat
				gcs.kalon = new_kalon
				gcs.kaalt = new_kaalt
			user_input = 0

		if int(user_input) == 5 or auto == True:
			auto = True
			try:
				user_input = autostack.pop()
			except:
				user_input = 0
				auto = False

			if user_input == 2:
				pass
				#wait until aircraft is approaching last waypoint of previous trajectory
				#current_wp = 0
				#mission.empty_socket()
				#while current_wp < 2: #start sending next mission half way through previous
					#current_wp = mission.master.waypoint_current()
					#print current_wp
					#time.sleep(1)


