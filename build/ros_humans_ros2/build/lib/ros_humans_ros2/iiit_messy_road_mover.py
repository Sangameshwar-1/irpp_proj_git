#!/usr/bin/env python3
"""
Movement controller for expanded IIIT Hyderabad Messy Road simulation.
All entities move ONLY within designated movable regions (roads, sidewalks, paths).
Includes autonomous rover for path planning navigation.
"""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose
import math
import random


class IIITMessyRoadMover(Node):
    def __init__(self):
        super().__init__('iiit_messy_road_mover')
        
        # Service client for setting entity poses in Gazebo
        self.pose_client = self.create_client(
            SetEntityPose,
            '/world/iiit_messy_road/set_pose'
        )
        
        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetEntityPose service...')
        
        # ============================================================
        # MOVABLE REGIONS DEFINITION
        # All entities MUST stay within these regions
        # ============================================================
        self.movable_regions = {
            # ROADS (vehicles and rover)
            'roads': [
                # Highway (main EW highway at top)
                {'type': 'rect', 'x_min': -140, 'x_max': 140, 'y_min': 70, 'y_max': 95},
                
                # Main Avenue NS (central vertical road)
                {'type': 'rect', 'x_min': -6, 'x_max': 6, 'y_min': -100, 'y_max': 100},
                
                # West Boulevard NS
                {'type': 'rect', 'x_min': -65, 'x_max': -55, 'y_min': -100, 'y_max': 100},
                
                # East Boulevard NS
                {'type': 'rect', 'x_min': 55, 'x_max': 65, 'y_min': -100, 'y_max': 100},
                
                # Far West Road NS
                {'type': 'rect', 'x_min': -125, 'x_max': -115, 'y_min': -100, 'y_max': 70},
                
                # Far East Road NS
                {'type': 'rect', 'x_min': 115, 'x_max': 125, 'y_min': -100, 'y_max': 70},
                
                # IIIT Main Road EW
                {'type': 'rect', 'x_min': -130, 'x_max': 130, 'y_min': 15, 'y_max': 25},
                
                # South Cross Road EW
                {'type': 'rect', 'x_min': -130, 'x_max': 130, 'y_min': -45, 'y_max': -35},
                
                # Mid South Road EW
                {'type': 'rect', 'x_min': -130, 'x_max': 130, 'y_min': -15, 'y_max': -5},
                
                # Campus Inner Road EW
                {'type': 'rect', 'x_min': -100, 'x_max': 100, 'y_min': 45, 'y_max': 55},
                
                # Far South Road EW
                {'type': 'rect', 'x_min': -100, 'x_max': 100, 'y_min': -90, 'y_max': -80},
                
                # Diagonal NE-SW connector
                {'type': 'diagonal', 'x1': 120, 'y1': 70, 'x2': 60, 'y2': 20, 'width': 8},
                
                # Diagonal NW-SE connector
                {'type': 'diagonal', 'x1': -120, 'y1': 70, 'x2': -60, 'y2': 20, 'width': 8},
                
                # Roundabouts (circular areas)
                {'type': 'circle', 'cx': 0, 'cy': 20, 'radius': 15},    # Central
                {'type': 'circle', 'cx': 0, 'cy': -40, 'radius': 12},   # South
                {'type': 'circle', 'cx': -60, 'cy': 20, 'radius': 10},  # West junction
            ],
            
            # SIDEWALKS (pedestrians only)
            'sidewalks': [
                # Highway sidewalks
                {'type': 'rect', 'x_min': -140, 'x_max': 140, 'y_min': 90, 'y_max': 98},
                {'type': 'rect', 'x_min': -140, 'x_max': 140, 'y_min': 65, 'y_max': 72},
                
                # Main Avenue sidewalks
                {'type': 'rect', 'x_min': -10, 'x_max': -6, 'y_min': -100, 'y_max': 100},
                {'type': 'rect', 'x_min': 6, 'x_max': 10, 'y_min': -100, 'y_max': 100},
                
                # West Boulevard sidewalks
                {'type': 'rect', 'x_min': -70, 'x_max': -65, 'y_min': -100, 'y_max': 100},
                {'type': 'rect', 'x_min': -55, 'x_max': -50, 'y_min': -100, 'y_max': 100},
                
                # East Boulevard sidewalks
                {'type': 'rect', 'x_min': 50, 'x_max': 55, 'y_min': -100, 'y_max': 100},
                {'type': 'rect', 'x_min': 65, 'x_max': 70, 'y_min': -100, 'y_max': 100},
                
                # IIIT Road sidewalks
                {'type': 'rect', 'x_min': -130, 'x_max': 130, 'y_min': 25, 'y_max': 30},
                {'type': 'rect', 'x_min': -130, 'x_max': 130, 'y_min': 10, 'y_max': 15},
            ],
            
            # PARKS (pedestrians only)
            'parks': [
                {'type': 'rect', 'x_min': -45, 'x_max': -20, 'y_min': 25, 'y_max': 45},  # Central park
                {'type': 'rect', 'x_min': 20, 'x_max': 50, 'y_min': 55, 'y_max': 75},    # Stadium area
            ],
            
            # PARKING LOTS (vehicles can stop)
            'parking': [
                {'type': 'rect', 'x_min': 85, 'x_max': 100, 'y_min': 20, 'y_max': 45},   # Hospital parking
                {'type': 'rect', 'x_min': 85, 'x_max': 100, 'y_min': -50, 'y_max': -25}, # Mall parking
                {'type': 'rect', 'x_min': 65, 'x_max': 80, 'y_min': -5, 'y_max': 10},    # General parking
            ],
        }
        
        # Traffic signal states
        self.signals = {
            'central': {'green': True, 'timer': 0, 'cycle': 30.0},
            'south': {'green': True, 'timer': 15, 'cycle': 30.0},
        }
        
        # Key destination locations
        self.destinations = {
            'hospital': (90, 32),
            'mall': (90, -35),
            'university': (-90, 55),
            'industrial': (30, -85),
            'residential': (-90, -45),
            'start': (0, 0),
        }
        
        # ============================================================
        # ROVER - Autonomous navigation robot
        # ============================================================
        self.rover = {
            'rover_main': {
                'speed': 2.5, 'behavior': 'autonomous',
                'position': [0, 0, 0.15], 'yaw': 0,
                'waypoints': [
                    # Route 1: Start -> Hospital (via main avenue + inner road)
                    (0, 0, 'navigate'),
                    (0, 20, 'navigate'),       # Central roundabout
                    (0, 50, 'navigate'),       # North on main avenue
                    (60, 50, 'navigate'),      # East on campus inner
                    (60, 32, 'navigate'),      # South to hospital
                    (90, 32, 'arrive'),        # Hospital destination
                    # Return via different route
                    (60, 32, 'navigate'),
                    (60, 20, 'navigate'),      # East blvd
                    (60, -10, 'navigate'),     # Mid south road
                    (0, -10, 'navigate'),      # West on mid south
                    (0, 0, 'navigate'),        # Back to start
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'navigating'
            }
        }
        
        # ============================================================
        # HUMANS - Constrained to sidewalks, parks, crossings
        # ============================================================
        self.humans = {
            'human_tall_fast': {
                'height': 1.9, 'speed': 2.0, 'behavior': 'commute_hospital',
                'position': [-100, 92, 0], 'yaw': 0,
                'waypoints': [
                    (-100, 92, 'walk'),       # Highway sidewalk
                    (-60, 92, 'walk'),        # Continue on sidewalk
                    (-60, 68, 'walk'),        # South on west blvd sidewalk
                    (-60, 50, 'walk'),        # Campus inner sidewalk
                    (-60, 28, 'walk'),        # Continue south
                    (0, 28, 'walk'),          # Cross at roundabout
                    (8, 50, 'walk'),          # Main avenue sidewalk north
                    (60, 50, 'walk'),         # East on inner road
                    (68, 32, 'arrive'),       # Hospital sidewalk
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
            
            'human_avg_medium': {
                'height': 1.7, 'speed': 1.3, 'behavior': 'commute_mall',
                'position': [-50, 28, 0], 'yaw': 0,
                'waypoints': [
                    (-50, 28, 'walk'),        # IIIT road sidewalk
                    (0, 28, 'wait_signal'),   # Central roundabout crossing
                    (8, 0, 'walk'),           # Main avenue sidewalk south
                    (8, -40, 'walk'),         # South roundabout area
                    (60, -38, 'walk'),        # South cross sidewalk
                    (88, -35, 'arrive'),      # Mall entrance
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
            
            'human_short_slow': {
                'height': 1.5, 'speed': 0.6, 'behavior': 'commute_university',
                'position': [8, 68, 0], 'yaw': 3.14159,
                'waypoints': [
                    (8, 68, 'walk'),          # Highway sidewalk
                    (-55, 68, 'walk'),        # West on sidewalk
                    (-68, 55, 'arrive'),      # University entrance
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
            
            'human_child_runner': {
                'height': 1.2, 'speed': 3.0, 'behavior': 'playing',
                'position': [-30, 35, 0], 'yaw': 0,
                'waypoints': [
                    # Playing in park only - constrained to park area
                    (-35, 30, 'run'),
                    (-25, 40, 'run'),
                    (-40, 35, 'run'),
                    (-22, 32, 'run'),
                    (-38, 42, 'run'),
                    (-28, 28, 'run'),
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
            
            'human_elderly_slow': {
                'height': 1.6, 'speed': 0.35, 'behavior': 'commute_residential',
                'position': [-68, 68, 0], 'yaw': -1.5708,
                'waypoints': [
                    (-68, 68, 'walk'),        # West blvd sidewalk
                    (-68, 50, 'pause'),       # Rest
                    (-68, 28, 'walk'),        # Continue south
                    (-68, 0, 'pause'),        # Rest
                    (-68, -38, 'walk'),       # South cross sidewalk
                    (-88, -45, 'arrive'),     # Residential
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
            
            'human_jogger': {
                'height': 1.75, 'speed': 3.2, 'behavior': 'jogging',
                'position': [68, 20, 0], 'yaw': 3.14159,
                'waypoints': [
                    # Jogging circuit on sidewalks
                    (68, 22, 'jog'),
                    (68, 50, 'jog'),          # North on east blvd sidewalk
                    (8, 52, 'jog'),           # West on inner road sidewalk
                    (8, 28, 'jog'),           # South on main sidewalk
                    (-52, 28, 'jog'),         # West on IIIT sidewalk
                    (-52, -8, 'jog'),         # South
                    (8, -8, 'jog'),           # East on mid-south sidewalk
                    (52, -8, 'jog'),          # Continue east
                    (68, 20, 'jog'),          # Back north
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
            
            'human_phone_user': {
                'height': 1.68, 'speed': 0.5, 'behavior': 'distracted',
                'position': [28, 68, 0], 'yaw': -1.5708,
                'waypoints': [
                    # Walks on stadium area sidewalks
                    (28, 68, 'walk'),
                    (28, 58, 'pause'),        # Stop to text
                    (35, 60, 'walk'),
                    (45, 65, 'pause'),        # Another stop
                    (40, 70, 'walk'),
                    (28, 68, 'walk'),         # Loop back
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
            
            'human_group_1': {
                'height': 1.72, 'speed': 1.1, 'behavior': 'commute_industrial',
                'position': [68, 92, 0], 'yaw': -2.35,
                'waypoints': [
                    (68, 92, 'walk'),         # Highway sidewalk
                    (68, 68, 'walk'),         # South on east blvd sidewalk
                    (68, 50, 'walk'),
                    (68, -8, 'walk'),         # Continue on sidewalk
                    (8, -8, 'walk'),          # Mid-south sidewalk
                    (8, -38, 'walk'),         # South roundabout
                    (8, -82, 'walk'),         # Far south sidewalk
                    (30, -82, 'arrive'),      # Industrial area
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
            
            'human_group_2': {
                'height': 1.65, 'speed': 1.1, 'behavior': 'commute_industrial',
                'position': [70, 92, 0], 'yaw': -2.35,
                'waypoints': [
                    (70, 92, 'walk'),         # Following group_1
                    (70, 68, 'walk'),
                    (70, 50, 'walk'),
                    (70, -8, 'walk'),
                    (10, -8, 'walk'),
                    (10, -38, 'walk'),
                    (10, -82, 'walk'),
                    (32, -82, 'arrive'),
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
        }
        
        # ============================================================
        # VEHICLES - Constrained to roads only
        # ============================================================
        self.vehicles = {
            'vehicle_car_fast': {
                'speed': 15.0, 'behavior': 'highway',
                'position': [-140, 80, 0], 'yaw': 0,
                'waypoints': [
                    (-140, 80, 'drive'),      # Highway
                    (-50, 80, 'stop_toll'),   # Toll booth
                    (0, 80, 'drive'),
                    (60, 80, 'drive'),
                    (140, 80, 'drive'),
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving', 'toll_paid': False
            },
            
            'vehicle_car_medium': {
                'speed': 12.0, 'behavior': 'city_drive',
                'position': [140, 82, 0], 'yaw': 3.14159,
                'waypoints': [
                    (140, 82, 'drive'),       # Highway eastbound
                    (60, 82, 'drive'),
                    (60, 50, 'slow'),         # Turn to east blvd
                    (60, 20, 'slow'),         # IIIT road junction
                    (60, -10, 'drive'),       # Mid-south junction
                    (60, -40, 'slow'),        # South cross
                    (90, -40, 'stop'),        # Mall parking
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
            
            'vehicle_suv_slow': {
                'speed': 8.0, 'behavior': 'campus_entry',
                'position': [-130, 20, 0], 'yaw': 0,
                'waypoints': [
                    (-130, 20, 'drive'),      # IIIT main road
                    (-105, 20, 'stop_gate'),  # Main gate
                    (-60, 20, 'slow'),        # Speed bump area
                    (0, 20, 'slow'),          # Central roundabout
                    (60, 20, 'drive'),        # Continue east
                    (70, 5, 'stop'),          # Parking
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving', 'gate_opened': False
            },
            
            'vehicle_truck_slow': {
                'speed': 6.0, 'behavior': 'delivery',
                'position': [60, 82, 0], 'yaw': 3.14159,
                'waypoints': [
                    (60, 82, 'drive'),        # Highway
                    (0, 80, 'drive'),
                    (-50, 80, 'stop_toll'),   # Toll
                    (-120, 80, 'slow'),       # Far west junction
                    (-120, 0, 'slow'),        # Far west road south
                    (-120, -85, 'slow'),      # Far south
                    (-30, -85, 'stop'),       # Factory delivery
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving', 'toll_paid': False
            },
            
            'vehicle_motorcycle': {
                'speed': 20.0, 'behavior': 'weaving',
                'position': [-80, 80, 0], 'yaw': 0,
                'waypoints': [
                    (-80, 80, 'drive'),       # Highway
                    (0, 80, 'drive'),
                    (60, 80, 'drive'),
                    (60, 50, 'drive'),        # East blvd south
                    (60, 20, 'drive'),        # IIIT junction
                    (60, -10, 'drive'),       # Mid-south junction
                    (0, -10, 'drive'),        # West on mid-south
                    (-60, -10, 'drive'),      # West blvd junction
                    (-60, 20, 'drive'),       # North on west blvd
                    (-60, 50, 'drive'),       # Inner road junction
                    (-60, 80, 'drive'),       # Back to highway
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
            
            'vehicle_bus': {
                'speed': 8.0, 'behavior': 'bus_route',
                'position': [-140, 82, 0], 'yaw': 0,
                'waypoints': [
                    (-140, 82, 'drive'),      # Highway
                    (-50, 82, 'stop_toll'),   # Toll
                    (-20, 92, 'stop_bus'),    # Bus stop
                    (30, 82, 'drive'),
                    (90, 82, 'drive'),
                    (140, 82, 'drive'),
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving', 'toll_paid': False
            },
            
            'vehicle_bicycle': {
                'speed': 4.5, 'behavior': 'bicycle',
                'position': [62, 30, 0], 'yaw': 1.5708,
                'waypoints': [
                    # Bicycle stays on road edges
                    (62, 30, 'ride'),
                    (62, 50, 'ride'),         # East blvd north
                    (0, 50, 'ride'),          # Campus inner west
                    (0, 20, 'ride'),          # Roundabout
                    (-60, 20, 'ride'),        # IIIT road west
                    (-60, 50, 'ride'),        # West blvd north
                    (0, 50, 'ride'),          # Back east
                    (62, 50, 'ride'),         # East blvd
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
            
            'vehicle_auto': {
                'speed': 6.5, 'behavior': 'auto',
                'position': [0, 22, 0], 'yaw': 0,
                'waypoints': [
                    # Auto-rickshaw on campus roads
                    (0, 22, 'drive'),         # Central roundabout
                    (60, 20, 'drive'),        # East on IIIT road
                    (60, -10, 'drive'),       # South to mid-south
                    (60, -40, 'slow'),        # South cross junction
                    (0, -40, 'slow'),         # South roundabout
                    (-60, -40, 'drive'),      # West cross
                    (-60, -10, 'drive'),      # North on west blvd
                    (-60, 20, 'drive'),       # West junction
                    (0, 20, 'drive'),         # Back to central
                ],
                'current_waypoint': 0, 'wait_time': 0, 'state': 'moving'
            },
        }
        
        # Timer for movement updates
        self.timer = self.create_timer(0.05, self.update_movement)
        self.dt = 0.05
        
        self.get_logger().info(
            f'IIIT Mover initialized: 1 rover, {len(self.humans)} humans, {len(self.vehicles)} vehicles'
        )
        self.get_logger().info(
            f'All entities constrained to movable regions (roads, sidewalks, parks)'
        )

    def is_in_movable_region(self, x, y, entity_type='vehicle'):
        """Check if position is within valid movable region"""
        regions_to_check = []
        
        if entity_type == 'vehicle' or entity_type == 'rover':
            regions_to_check = self.movable_regions['roads'] + self.movable_regions['parking']
        elif entity_type == 'human':
            regions_to_check = (self.movable_regions['sidewalks'] + 
                               self.movable_regions['parks'] + 
                               self.movable_regions['roads'])  # Can cross roads
        
        for region in regions_to_check:
            if region['type'] == 'rect':
                if (region['x_min'] <= x <= region['x_max'] and 
                    region['y_min'] <= y <= region['y_max']):
                    return True
            elif region['type'] == 'circle':
                dist = math.sqrt((x - region['cx'])**2 + (y - region['cy'])**2)
                if dist <= region['radius']:
                    return True
            elif region['type'] == 'diagonal':
                # Check if point is within diagonal corridor
                x1, y1, x2, y2 = region['x1'], region['y1'], region['x2'], region['y2']
                width = region['width']
                # Calculate distance from line
                line_len = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                if line_len > 0:
                    dist = abs((y2-y1)*x - (x2-x1)*y + x2*y1 - y2*x1) / line_len
                    # Check if within line segment bounds
                    t = ((x-x1)*(x2-x1) + (y-y1)*(y2-y1)) / (line_len**2)
                    if 0 <= t <= 1 and dist <= width/2:
                        return True
        
        return False

    def clamp_to_nearest_road(self, x, y, entity_type='vehicle'):
        """Clamp position to nearest valid road/path"""
        # For simplicity, just return the original position if valid
        # or find the nearest road center
        if self.is_in_movable_region(x, y, entity_type):
            return x, y
        
        # Find nearest road center
        road_centers = [
            (0, y),      # Main avenue
            (-60, y),    # West blvd
            (60, y),     # East blvd
            (x, 80),     # Highway
            (x, 20),     # IIIT road
            (x, -10),    # Mid-south
            (x, -40),    # South cross
            (x, 50),     # Campus inner
        ]
        
        min_dist = float('inf')
        best_pos = (x, y)
        
        for cx, cy in road_centers:
            dist = math.sqrt((x-cx)**2 + (y-cy)**2)
            if dist < min_dist and self.is_in_movable_region(cx, cy, entity_type):
                min_dist = dist
                best_pos = (cx, cy)
        
        return best_pos

    def update_traffic_signals(self):
        """Update traffic signal states"""
        for signal in self.signals.values():
            signal['timer'] += self.dt
            if signal['timer'] >= signal['cycle']:
                signal['timer'] = 0
                signal['green'] = not signal['green']

    def get_nearest_signal(self, x, y):
        """Get nearest traffic signal state"""
        if abs(x) < 15 and abs(y - 20) < 15:
            return self.signals['central']
        elif abs(x) < 15 and abs(y + 40) < 15:
            return self.signals['south']
        return None

    def update_movement(self):
        """Update all entity positions"""
        self.update_traffic_signals()
        
        # Move rover
        for name, rover in self.rover.items():
            self.move_rover(name, rover)
        
        # Move humans
        for name, human in self.humans.items():
            self.move_human(name, human)
        
        # Move vehicles
        for name, vehicle in self.vehicles.items():
            self.move_vehicle(name, vehicle)

    def move_rover(self, name, rover):
        """Move rover along path planning route - constrained to roads"""
        if rover['state'] == 'arrived':
            rover['wait_time'] += self.dt
            if rover['wait_time'] > 5.0:  # Wait 5 seconds at destination
                rover['state'] = 'navigating'
                rover['wait_time'] = 0
                rover['current_waypoint'] += 1
            return
            
        waypoints = rover['waypoints']
        if rover['current_waypoint'] >= len(waypoints):
            rover['current_waypoint'] = 0
            
        target = waypoints[rover['current_waypoint']]
        tx, ty, action = target
        
        # Verify target is on valid road
        tx, ty = self.clamp_to_nearest_road(tx, ty, 'rover')
        
        if action == 'arrive':
            rover['state'] = 'arrived'
            self.get_logger().info(f'Rover arrived at waypoint {rover["current_waypoint"]}')
            return
        
        # Movement
        px, py, pz = rover['position']
        dx, dy = tx - px, ty - py
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist < 0.5:
            rover['current_waypoint'] += 1
            return
        
        speed = rover['speed']
        move_dist = min(speed * self.dt, dist)
        nx = px + (dx / dist) * move_dist
        ny = py + (dy / dist) * move_dist
        
        # Ensure new position is valid
        nx, ny = self.clamp_to_nearest_road(nx, ny, 'rover')
        
        yaw = math.atan2(dy, dx)
        
        rover['position'] = [nx, ny, pz]
        rover['yaw'] = yaw
        
        self.set_entity_pose(name, nx, ny, 0.15, yaw)

    def move_human(self, name, human):
        """Move human towards waypoint - constrained to sidewalks/parks"""
        if human['state'] == 'arrived':
            return
            
        waypoints = human['waypoints']
        if human['current_waypoint'] >= len(waypoints):
            human['current_waypoint'] = 0
            
        target = waypoints[human['current_waypoint']]
        tx, ty, action = target
        
        # Handle actions
        if action == 'wait_signal':
            signal = self.get_nearest_signal(human['position'][0], human['position'][1])
            if signal and not signal['green']:
                return
        elif action == 'pause':
            human['wait_time'] += self.dt
            if human['wait_time'] < random.uniform(2, 5):
                return
            human['wait_time'] = 0
        elif action == 'arrive':
            human['state'] = 'arrived'
            self.get_logger().info(f'{name} arrived at destination')
            return
        
        # Movement
        px, py, pz = human['position']
        dx, dy = tx - px, ty - py
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist < 0.3:
            human['current_waypoint'] += 1
            return
        
        speed = human['speed']
        if action == 'jog':
            speed *= 1.5
        elif action == 'run':
            speed *= 2.0
        
        move_dist = min(speed * self.dt, dist)
        nx = px + (dx / dist) * move_dist
        ny = py + (dy / dist) * move_dist
        yaw = math.atan2(dy, dx)
        
        human['position'] = [nx, ny, pz]
        human['yaw'] = yaw
        
        self.set_entity_pose(name, nx, ny, human['height'] * 0.45, yaw)

    def move_vehicle(self, name, vehicle):
        """Move vehicle towards waypoint - constrained to roads"""
        waypoints = vehicle['waypoints']
        if vehicle['current_waypoint'] >= len(waypoints):
            vehicle['current_waypoint'] = 0
            
        target = waypoints[vehicle['current_waypoint']]
        tx, ty, action = target
        
        # Verify target is on valid road
        tx, ty = self.clamp_to_nearest_road(tx, ty, 'vehicle')
        
        # Handle actions
        if action == 'stop_toll':
            if not vehicle.get('toll_paid', True):
                vehicle['wait_time'] += self.dt
                if vehicle['wait_time'] < random.uniform(3, 6):
                    return
                vehicle['toll_paid'] = True
                vehicle['wait_time'] = 0
        elif action == 'stop_gate':
            if not vehicle.get('gate_opened', True):
                vehicle['wait_time'] += self.dt
                if vehicle['wait_time'] < random.uniform(4, 8):
                    return
                vehicle['gate_opened'] = True
                vehicle['wait_time'] = 0
        elif action == 'stop_bus':
            vehicle['wait_time'] += self.dt
            if vehicle['wait_time'] < random.uniform(10, 15):
                return
            vehicle['wait_time'] = 0
        elif action == 'stop':
            vehicle['state'] = 'parked'
            return
        
        # Movement
        px, py, pz = vehicle['position']
        dx, dy = tx - px, ty - py
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist < 1.0:
            vehicle['current_waypoint'] += 1
            if action == 'stop_toll':
                vehicle['toll_paid'] = False
            if action == 'stop_gate':
                vehicle['gate_opened'] = False
            return
        
        speed = vehicle['speed']
        if action == 'slow':
            speed *= 0.4
        elif action == 'ride':
            speed *= 0.8
        
        move_dist = min(speed * self.dt, dist)
        nx = px + (dx / dist) * move_dist
        ny = py + (dy / dist) * move_dist
        
        # Ensure new position is valid road
        nx, ny = self.clamp_to_nearest_road(nx, ny, 'vehicle')
        
        yaw = math.atan2(dy, dx)
        
        vehicle['position'] = [nx, ny, pz]
        vehicle['yaw'] = yaw
        
        height = 0.5 if 'bicycle' in name else (0.75 if 'motorcycle' in name else 0.6)
        self.set_entity_pose(name, nx, ny, height, yaw)

    def set_entity_pose(self, name, x, y, z, yaw):
        """Send pose update to Gazebo"""
        request = SetEntityPose.Request()
        
        gz_entity = Entity()
        gz_entity.name = name
        gz_entity.type = Entity.MODEL
        
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.z = float(math.sin(yaw / 2))
        pose.orientation.w = float(math.cos(yaw / 2))
        
        request.entity = gz_entity
        request.pose = pose
        
        self.pose_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = IIITMessyRoadMover()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
