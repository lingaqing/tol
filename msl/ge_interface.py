from fastkml import kml, styles
from fastkml.geometry import Geometry, Point, LineString, Polygon
from pymavlink import mavutil
import time
from threading import Thread
import pdb
from math import pi, atan2, sin, cos, sqrt

class ge_interface(Thread):

	def __init__(self,address,kaloc):
		super(ge_interface, self).__init__()
		self.master = mavutil.mavlink_connection(address)
		self.lat = 'unknown'
		self.lon = 'unknown'
		self.alt = 'unknown'
		self.hdg = 'unknown'
		self.kalat = kaloc[0]
		self.kalon = kaloc[1]
		self.kaalt = kaloc[2]
		self.east = 'unknown'
		self.north = 'unknown'
		self.up = 'unknown'
		self.poison = False

	def run(self):
		while True:
			#poison pill check
			if self.poison == True:
				break
			else:
				#read gps data (don't block)	
				msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
				if msg:
					#print msg
					self.aclat = msg.lat/1e7
					self.aclon = msg.lon/1e7
					self.acalt = msg.alt/1e3
					self.achdg = msg.hdg/1e2

					#Find position relative to Ka-1 (ENU)
					#Convert degrees to radians:
					lat1 = self.kalat * pi/180
					lon1 = self.kalon * pi/180
					lat2 = self.aclat * pi/180
					lon2 = self.aclon * pi/180
					dlat  = lat2-lat1
					dlong = lon2-lon1

					#Haversine formula
					R = 6371000
					a = sin(dlat/2)*sin(dlat/2) + cos(lat1)*cos(lat2)*sin(dlong/2.0)*sin(dlong/2.0)
					c = 2.0 * atan2( sqrt(a), sqrt(1-a) )
					d = R * c;	#distance
					b = atan2( sin(dlong)*cos(lat2), cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dlong) )

					self.east = d*cos(pi/2-b)	#x meters offset from Datum (ENU)
					self.north = d*sin(pi/2-b)	#y meters offset from Datum (ENU)
					self.up = self.acalt - self.kaalt #z meters offset from Datum (ENU), should never be negative lol

					#write KML for plane
					ns = '{http://www.opengis.net/kml/2.2}'
					d = kml.Document(ns=ns, name='TOL GCS')
					k = kml.KML(ns=ns)

					#AIRCRAFT
					p= kml.Placemark(ns, name='sUAS(' + '{0:.2f}'.format(self.east) +','+ '{0:.2f}'.format(self.north) +','+ '{0:.2f}'.format(self.up) + ')', styleUrl='sn_airports')
					#AC Style
					s = styles.Style(id='sn_airports')
					IS = styles.IconStyle(scale=1.2, icon_href='http://maps.google.com/mapfiles/kml/shapes/airports.png', heading=self.achdg)
					s.append_style(IS)
					#AC Geometry
					geom = Geometry()
					geom.geometry = Point(self.aclon, self.aclat, self.acalt)
					geom.altitude_mode = 'absolute'
					p.geometry = geom
					d.append_style(s)
					d.append(p)

					#MGS
					p = kml.Placemark(ns, name='MGS (0,0,0)',styleUrl='sn_truck')
					s = styles.Style(id='sn_truck')
					IS = styles.IconStyle(scale=1.2, icon_href='http://maps.google.com/mapfiles/kml/shapes/truck.png')
					s.append_style(IS)
					#MGS Geometry
					geom = Geometry()
					geom.geometry = Point(self.kalon, self.kalat, self.kaalt)
					geom.altitude_mode = 'absolute'
					p.geometry = geom
					d.append_style(s)
					d.append(p)

					#WRITE
					k.append(d)
					kmlfile = open('TOL_GCS.kml',"w")
					kmlfile.write(k.to_string(prettyprint=True))
					kmlfile.close()
