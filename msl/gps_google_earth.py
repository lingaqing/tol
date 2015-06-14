from fastkml import kml, styles
from fastkml.geometry import Geometry, Point, LineString, Polygon
from pymavlink import mavutil
import time

class ge_interface:

	master = mavutil.mavlink_connection('udp:localhost:14550')
	timeout = 5 #seconds
	t0 =  time.time()
	msg = None
	while time.time() - t0 < timeout and msg == None:
		msg = master.wait_heartbeat(blocking=False)
	if msg == None:
		#raise StandardError('Cannot communicate with the aircraft.')
		print ('Cannot communicate with the aircraft. Please check aircraft address.')
	else:
		while True:
			msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
			lat = msg.lat/1e7
			lon = msg.lon/1e7
			alt = msg.alt/1e3
			hdg = msg.hdg/1e2

			#write KML for plane
			ns = '{http://www.opengis.net/kml/2.2}'
			d = kml.Document(ns=ns, name='sUAS GPS')
			k = kml.KML(ns=ns)
			p = kml.Placemark(ns, 'id', 'sUAS', 'description',styleUrl='sn_airports')

			#Style
			s = styles.Style(id='sn_airports')
			IS = styles.IconStyle(scale=1.2, icon_href='http://maps.google.com/mapfiles/kml/shapes/airports.png', heading=hdg)
			s.append_style(IS)

			#Geometry
			geom = Geometry()
			geom.geometry = Point(lon, lat, alt)
			geom.altitude_mode = 'absolute'
			p.geometry = geom

			d.append_style(s)
			d.append(p)
			k.append(d)
			kmlfile = open('sUAS.kml',"w")
			kmlfile.write(k.to_string(prettyprint=True))
			kmlfile.close()