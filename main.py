import glob
import os
import sys

tmp = ['/home/zz/carmaker_ros/ros/ros1_ws/devel/lib/python2.7/dist-packages',
       '/opt/ros/ros1/lib/python2.7/dist-packages', '/opt/ros/kinetic/lib/python2.7/dist-packages',
       '/home/zz/Downloads/CARLA_0.9.6/PythonAPI/carla-0.9.6-py3.5-linux-x86_64.egg',
       '/home/zz/Downloads/CARLA_0.9.6/PythonAPI/carla/dist/carla-0.9.6-py3.5-linux-x86_64.egg_FILES']
for index in range(0, len(tmp)):
    if tmp[index] in sys.path:
        sys.path.remove(tmp[index])

if not ('/home/zz/Downloads/CARLA_0.9.6/PythonAPI/carla/dist/carla-0.9.6-py3.5-linux-x86_64.egg_FILES' in sys.path):
    sys.path.append('/home/zz/Downloads/CARLA_0.9.6/PythonAPI/carla/dist/carla-0.9.6-py3.5-linux-x86_64.egg_FILES')

if not ('/home/zz/Downloads/CARLA_0.9.6/HDMaps' in sys.path):
    sys.path.append('/home/zz/Downloads/CARLA_0.9.6/HDMaps')
if not ('/home/zz/Downloads/CARLA_0.9.6/PythonAPI/carla' in sys.path):
    sys.path.append('/home/zz/Downloads/CARLA_0.9.6/PythonAPI/carla')


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from sensor_class import *
from KeyboardShortCutSetting import *
import random
from HUD import HUD

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

from pygame import gfxdraw

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue
import cv2

class CarlaEnv():
    pygame.init()
    font = pygame.font.init()

    def __init__(self,world):
        #화면 크기
        self.width = 800
        self.height = 600
        #센서 종류
        self.actor_list = []
        self.vehicle =None
        self.camera_rgb = None
        self.camera_semseg = None
        self.lane_invasion_sensor = None
        self.collision_sensor = None
        self.gnss_sensor = None
        self.waypoint = None


        self.world=world
        self.map = world.get_map()
        self.spectator = self.world.get_spectator()
        self.hud = HUD(self.width, self.height)
        self.waypoints = self.map.generate_waypoints(1.0)
        for p in self.waypoints:
            world.debug.draw_string(p.transform.location, 'o', draw_shadow=True,
                                    color=carla.Color(r=255, g=255, b=255), life_time=5)
        # self.waypoint_tuple_list = self.map.get_topology()ds
        # print(len(self.waypoint_tuple_list))
        # print(self.waypoint_tuple_list[0][0].transform)

        self.restart()
        self.main()

    def restart(self):
        blueprint_library = world.get_blueprint_library()
        start_pose = random.choice(self.map.get_spawn_points())
        self.waypoint = self.map.get_waypoint(start_pose.location)

        # print(start_pose)
        # print(self.waypoint.transform)
        self.spectator.set_transform(carla.Transform(start_pose.location + carla.Location(z=10), carla.Rotation(pitch=-90)))

        self.vehicle = world.spawn_actor(
            random.choice(blueprint_library.filter('vehicle.bmw.grandtourer')),
            start_pose)
        self.actor_list.append(self.vehicle)
        # vehicle.set_simulate_physics(False)

        self.camera_rgb =RGBSensor(self.vehicle, self.hud)
        self.actor_list.append(self.camera_rgb.sensor)

        self.camera_semseg = SegmentationCamera(self.vehicle,self.hud)
        self.actor_list.append(self.camera_semseg.sensor)

        self.collision_sensor = CollisionSensor(self.vehicle, self.hud)  # 충돌 여부 판단하는 센서
        self.lane_invasion_sensor = LaneInvasionSensor(self.vehicle, self.hud)  # lane 침입 여부 확인하는 센서
        self.gnss_sensor = GnssSensor(self.vehicle)

    def draw_image(self, surface, image, blend=False):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if blend:
            image_surface.set_alpha(100)
        surface.blit(image_surface, (0, 0))

    def main(self):

        clock = pygame.time.Clock()


        display = pygame.display.set_mode(
            (self.width, self.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        Keyboardcontrol = KeyboardControl(self, False)
        # Controller = Controller(self.vehicle)
        try:
            # Create a synchronous mode context.
            with CarlaSyncMode(world, self.camera_rgb.sensor, self.camera_semseg.sensor, fps=30) as sync_mode:
                while True:
                    if Keyboardcontrol.parse_events(client, self, clock):
                        return

                    # clock.tick()
                    self.hud.tick(self, clock)

                    # Advance the simulation and wait for the data.
                    snapshot, image_rgb, image_semseg = sync_mode.tick(timeout=2.0)

                    # Choose the next waypoint and update the car location.
                    self.waypoint = random.choice(self.waypoint.next(1.5))
                    self.vehicle.set_transform(self.waypoint.transform)
                    self.waypoint = random.choice(self.waypoint.next(3))
                    self.world.debug.draw_point(self.waypoint.transform.location,size=0.1,color=carla.Color(r=255, g=255, b=255),life_time=1)
                    self.spectator.set_transform(
                        carla.Transform(self.waypoint.transform.location + carla.Location(z=40), carla.Rotation(pitch=-90)))

                    image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                    # fps = round(1.0 / snapshot.timestamp.delta_seconds)

                    # Draw the display.
                    self.draw_image(display, image_rgb)
                    self.draw_image(display, image_semseg, blend=False)
                    # pygame.gfxdraw.filled_circle(display,int(self.waypoint.transform.location.x),int(self.waypoint.transform.location.y),5,(255,255,255))
                    self.hud.render(display)

                    pygame.display.flip()

        finally:

            print('destroying actors.')
            for actor in self.actor_list:
                actor.destroy()

            pygame.quit()
            print('done.')






if __name__ == '__main__':

    try:

        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        CarlaEnv(world)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
