## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):    
    obj = bld.create_ns3_program('vanet-highway-test', ['core'])
    obj.source = [
	'vanet-highway-test.cc',
	'Highway.cc',
	'Controller.cc',
	'Vehicle.cc',
	'Obstacle.cc',
	'Model.cc',
	'LaneChange.cc',
	]
    obj2 = bld.create_ns3_program('vanet-highway-test-thomas', ['core'])
    obj2.source = [
	'vanet-highway-test-thomas.cc',
	'Highway.cc',
	'Controller.cc',
	'Vehicle.cc',
	'Obstacle.cc',
	'Model.cc',
	'LaneChange.cc',
	]
    obj3 = bld.create_ns3_program('vanet-highway-scenario2', ['core'])
    obj3.source = [
	'vanet-highway-scenario2.cc',
	'Highway.cc',
	'Controller.cc',
	'Vehicle.cc',
	'Obstacle.cc',
	'Model.cc',
	'LaneChange.cc',
	]



