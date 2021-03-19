import numpy as np
import math
from RRT_starPlanner import RRTStarPlanner
from map import *


def dist(pos1, pos2):
    # return math.sqrt(pow(pos1[0] - pos2[0], 2) + pow(pos1[1] - pos2[1], 2))
    return math.sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2))


class UAV:
    def __init__(self, env_map, Id, start_pos, end_pos, epsilon=0.2, step_size=10, uav_size=2, max_dist=20):
        self.Id = Id
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.map = env_map
        self.curr_position = self.start_pos  # current position of the uav in the map
        self.radius = uav_size  # size of the uav (used as safe distance to avoid collision)
        self.max_connect_dist = max_dist  # max distance btw two uavs for maintained connectivity

        # Control the planner's step towards q_rand from q_new and the prob of sampling the target (bias sampling)
        self.epsilon = epsilon
        self.stepSize = step_size

        self.planner = RRTStarPlanner(
            env_map,
            start_pos=self.start_pos,
            end_pos=self.end_pos,
            epsilon=self.epsilon,
            stepSize=self.stepSize
        )
        self.new_path = []  # path from resulting from planning going from the current location
        self.step = 0

        # path followed by the uav: when better paths are found the final is pruned and the better one is appended
        self.planner.plan()
        self.final_path = self.planner.finalPath

    def new_sub_target(self, end):
        self.start_pos = self.curr_position
        self.end_pos = end

        self.planner = RRTStarPlanner(
            self.map,
            start_pos=self.start_pos,
            end_pos=self.end_pos,
            epsilon=self.epsilon,
            stepSize=self.stepSize
        )
        self.planner.plan()
        path = self.planner.finalPath
        print('***-------********----- LENGTH PATH FOUND:', len(path))  # ***************************
        try:
            del self.final_path[self.step+1:]
            for pt in path:
                self.final_path.append(pt)
        except:
            for pt in path:
                self.final_path.append(pt)

    def move(self, all_uavs_position):
        if self.curr_position != self.start_pos:
            self.map.remove_dynamic_obstacle(self.Id)
            for key in all_uavs_position.keys():
                no_connection = self.check_connection(all_uavs_position)
                if key != self.Id and (self.check_collision(all_uavs_position) or no_connection):
                    self.planner.plan(start=self.curr_position, target=self.end_pos)
                    nb_tries = 0
                    while self.check_collision(all_uavs_position) or self.check_connection(all_uavs_position):
                        nb_tries += 1
                        print(
                            '+*+*+*####### COLLISION HAPPENED! ***************')  # ******************************************
                        print('** Uavs collision', self.check_collision(all_uavs_position),
                              '  Connection our of range:', self.check_connection(all_uavs_position))
                        # print(f'in while for uav {self.Id}')
                        self.planner = RRTStarPlanner(
                            self.map,
                            start_pos=self.curr_position,
                            end_pos=self.end_pos,
                            epsilon=self.epsilon, stepSize=self.stepSize
                        )

                        # self.planner.plan(start=self.curr_position, target=self.end_pos)
                        self.planner.plan(start=self.final_path[self.step - 2], target=self.end_pos)

                        ###
                        new_path = self.planner.finalPath
                        del self.final_path[self.step - 2:]
                        # print('>>>>>> length of final path after delete:', len(self.final_path))
                        for point in new_path:
                            self.final_path.append(point)

                        if nb_tries >= 25:
                            return

                    # self.step += 1
                    # self.curr_position = self.final_path[self.step]
                    if self.step + 1 < len(self.final_path):
                        self.step += 1
                        self.curr_position = self.final_path[self.step]

                else:
                    # print('len(self.final_path):', len(self.final_path), 'step:', self.step)
                    if self.step + 1 < len(self.final_path):
                        self.step += 1
                        self.curr_position = self.final_path[self.step]

            self.map.add_dynamic_obstacle(
                self.Id,
                CircleObstacle(
                    pos=Point(self.curr_position.x, self.curr_position.y),
                    radius=self.radius
                )
            )
        elif self.curr_position == self.end_pos:
            return
        else:
            # self.step += 1
            if self.step + 1 < len(self.final_path):
                self.step += 1
                self.curr_position = self.final_path[self.step]

    def check_point_collision(self, point1, point2):
        d = dist(point1, point2)
        if d < 2 * self.radius:
            return True
        return False

    def check_collision(self, all_uavs_next_pos):
        for key in all_uavs_next_pos.keys():
            if key != self.Id and self.step + 1 < len(self.final_path):
                d = dist(self.final_path[self.step + 1], all_uavs_next_pos[key])
                if d < 2 * self.radius:
                    return True
        return False

    # # check if the distance between me and all agents is greater than the connection distance
    # def check_connection(self, all_others_pos):
    #     for key in all_others_pos:
    #         if key != self.Id and ( self.step + 1 < len(self.final_path) or self.step - 1 == len(self.final_path) ):
    #             d = dist(self.final_path[self.step + 1], all_others_pos[key])
    #             if d <= self.max_connect_dist:
    #                 return False
    #     return True

    # check if the distance between me and all agents is greater than the connection distance
    def check_connection(self, all_others_pos):
        for key in all_others_pos:
            # if key != self.Id and ( self.step + 1 < len(self.final_path) or self.step - 1 == len(self.final_path) ):
            if key != self.Id and self.step + 1 < len(self.final_path):
                d = dist(self.final_path[self.step+1], all_others_pos[key])
                if d <= self.max_connect_dist:
                    return False
            elif key != self.Id and self.step == len(self.final_path) - 1:
                d = dist(self.final_path[self.step], all_others_pos[key])
                if d <= self.max_connect_dist:
                    return False
        return True
