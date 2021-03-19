import numpy as np
import math
from RRT_starPlanner import RRTStarPlanner


def dist(pos1, pos2):
    return math.sqrt(pow(pos1[0] - pos2[0], 2) + pow(pos1[1] - pos2[1], 2))

class UAV:
    def __init__(self, env_map, Id, start_pos, end_pos, planner, uav_size=10, max_dist=50):
        self.Id = Id
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.curr_position = self.start_pos  # current position of the uav in the map
        self.radius = uav_size  # size of the uav (used as safe distance to avoid collision)
        self.max_connect_dist = max_dist  # max distance btw two uavs for maintained connectivity
        self.planner = RRTStarPlanner(env_map, start_pos=self.start_pos, end_pos=self.end_pos, epsilon=0.2, stepSize=10)
        self.new_path = []  # path from resulting from planning going from the current location
        self.step = 0

        # path followed by the uav: when better paths are found the final is pruned and the better one is appended
        self.final_path = self.planner.plan()


    def uav_plan(self, all_uavs_plan):
        good_plan = True  # used for stopping condition of the recursive alg when no more violation is found on the path

        # check if self is the first to plan
        if len(all_uavs_plan) != 0:
            for i in range(len(self.final_path)):
                position_all_uavs_step_i = []
                for k in all_uavs_plan.keys():
                    if k != self.Id:
                        try:
                            pos_uav_k = all_uavs_plan[k][i]
                            position_all_uavs_step_i.append(pos_uav_k)
                        except:
                            continue
                no_connection = self.check_connection(self.final_path[i], position_all_uavs_step_i)
                for key in all_uavs_plan.keys():
                    if key != self.Id:
                        try:
                            is_collision = self.check_point_collision(self.final_path[i], all_uavs_plan[key][i])
                            if is_collision or no_connection:
                                remaining_path = self.planner.plan(start=self.final_path[i])
                                del self.final_path[i:]
                                for j in remaining_path:
                                    self.final_path.append(j)
                                all_uavs_plan[self.Id] = self.final_path
                                good_plan = self.uav_plan(all_uavs_plan)
                                if good_plan:
                                    return True
                            else:
                                continue

                        except:
                            continue

        else:
            all_uavs_plan[self.Id] = self.final_path


    def move(self):


    def check_point_collision(self, point1, point2):
        d = dist(point1, point2)
        if d < 2 * self.radius:
            return True
        return False

    def check_collision(self, all_uavs_next_pos):
        for key in all_uavs_next_pos.keys():
            if key != self.Id:
                d = dist(self.final_path[self.step+1], all_uavs_next_pos[key])
                if d < 2 * self.radius:
                    return True
        return False

    # check if the distance between me and all agents is greater than the connection distance
    def check_connection(self, pos, all_others_pos):
        for p in all_others_pos:
            if dist(pos, p) < self.max_connect_dist:
                return False
        return True


