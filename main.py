from RRT_starPlanner import RRTStarPlanner
from uav import UAV
from map import *
import time
import random
import math


def is_point_explored(point, uavs_visited_points, uavs_vision_range):
    """
    check if the provided point has been explored by any uav before
    uavs_visted_points: used to avoid sampling points that are all already visited by the uavs
    uavs_vision_range: max distance that a uav can see (used to evaluate the area explored given the position of uavs)
    """
    for pt in uavs_visited_points:
        if point.dist(pt) < uavs_vision_range:
            return True
    return False


def generate_sub_targets(env_map, any_uav, uavs_visited_points, n_pts, uavs_vision_range):
    """
    Generate sub targets for the uavs to plan to get to while looking for the real target (search and rescue)
    env_map: map of the environment
    uavs_visted_points: used to avoid sampling points that are all already visited by the uavs
    n_pts: number of target points to sample (equl to the number of uavs)
    uavs_vision_range: max distance that a uav can see (used to evaluate the area explored given the position of uavs)
    """
    point1 = Point(
        env_map.left + env_map.length * random.random(),
        env_map.down + env_map.width * random.random()
    )
    while any_uav.planner.check_obstacle(env_map, point1) or \
            is_point_explored(point1, uavs_visited_points, uavs_vision_range) or \
            env_map.out_of_map(point1):
        point1 = Point(
            env_map.left + env_map.length * random.random(),
            env_map.down + env_map.width * random.random()
        )
    all_sub_targets = [point1]
    for i in range(n_pts - 1):
        point = Point(
            all_sub_targets[i].x + math.sqrt(2) * uavs_vision_range * random.random(),
            all_sub_targets[i].y + math.sqrt(2) * uavs_vision_range * random.random()
        )
        while any_uav.planner.check_obstacle(env_map, point) or env_map.out_of_map(point):
            point = Point(
                all_sub_targets[i].x + math.sqrt(2) * uavs_vision_range * random.random(),
                all_sub_targets[i].y + math.sqrt(2) * uavs_vision_range * random.random()
            )
        all_sub_targets.append(point)
    return all_sub_targets


def generate_initial_start_end_pos(env_map, n_pts, max_dist, uav_size, which='start'):
    """
    Generate the starting positions for the uavs
    """
    if which == 'start':
        all_start_points = [Point(10, 10)]
    else:
        all_start_points = [Point(620, 420)]
    for i in range(n_pts - 1):
        point = Point(
            all_start_points[i].x + math.sqrt(2) * max_dist * random.random(),
            all_start_points[i].y + math.sqrt(2) * max_dist * random.random()
        )
        while all_start_points[i].dist(point) <= 3 * uav_size or \
                all_start_points[i].dist(point) >= max_dist or env_map.out_of_map(point):
            point = Point(
                all_start_points[i].x + math.sqrt(2) * max_dist * random.random(),
                all_start_points[i].y + math.sqrt(2) * max_dist * random.random()
            )
        all_start_points.append(point)
        if which == 'start':
            for pt1 in all_start_points:
                for pt2 in all_start_points:
                    if pt1 != pt2 and pt1.dist(pt2) <= 3 * uav_size:
                        all_start_points = generate_initial_start_end_pos(env_map, n_pts, max_dist, uav_size, which='start')

    return all_start_points


def main(nbr_uavs=4):
    map = Map(480, 0, 0, 640)
    size_uav = 8  # size of the uav (used to check for collision between the uavs)
    com_range_uav = 40  # communication range of the uavs
    uav_vision_range = 40

    # starts = [Point(0, 0), Point(0, 400), Point(320, 0)]
    # ends = [Point(640, 480), Point(640, 0), Point(0, 200)]
    # ends = [Point(0, 0), Point(0, 80), Point(60, 0)]
    # starts = [Point(640, 480), Point(640, 400), Point(560, 480)]

    starts = [Point(0, 0), Point(0, 40), Point(40, 0), Point(0, 70), Point(70, 0)]
    ends = [Point(610, 450), Point(620, 420), Point(600, 430), Point(640, 420), Point(600, 460)]

    map.add_obstacle(CircleObstacle(pos=Point(400, 300), radius=50))
    # map.add_obstacle(RectangleObstacle(top=200, down=100, left=100, right=300))
    map.add_obstacle(RectangleObstacle(top=100, down=50, left=450, right=550))
    map.add_obstacle(RectangleObstacle(top=200, down=150, left=50, right=250))
    map.add_obstacle(RectangleObstacle(top=400, down=300, left=50, right=200))
    uavs = {}
    rrt_start_planner = {}

    # starts = generate_initial_start_end_pos(map, nbr_uavs, com_range_uav, size_uav, which='start')
    # print('Initial positions generated! \n', starts)
    # ends = generate_initial_start_end_pos(map, nbr_uavs, com_range_uav, size_uav, which='end')

    path_colors = [(100, 0, 0), (0, 100, 0), (0, 100, 100), (100, 100, 0)]
    if nbr_uavs > 4:
        for u in range(4, nbr_uavs):
            col = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            path_colors.append(col)

    plan_all_uavs = {}
    visited_pts = []
    sub_target_reached = []
    for i in range(nbr_uavs):
        # rrtPlanners[i] = RRTStarPlanner(map, start_pos=starts[i], end_pos=ends[i], epsilon=0.2, stepSize=10)
        # uavs[i] = UAV(map, i, starts[i], ends[i], rrt_start_planner[i], uav_size=10, max_dist=50)
        uavs[i] = UAV(
            map, i, starts[i], ends[i], epsilon=0.1,
            step_size=5, uav_size=size_uav, max_dist=com_range_uav * 2
        )
        plan_all_uavs[i] = uavs[i].final_path
        sub_target_reached.append(False)

    # Generate the mission target for Search and Rescue mission
    target_point = Point(uavs[0].map.left + uavs[0].map.length * random.uniform(0.3, 0.999),
                         uavs[0].map.down + uavs[0].map.width * random.uniform(0.3, 0.999))
    # while uavs[0].planner.check_obstacle(map, target_point) or not is_in_the_map(map, target_point):
    while uavs[0].planner.check_obstacle(map, target_point) or map.out_of_map(target_point):
        print('TARGET NOT VALID ***************')
        target_point = Point(uavs[0].map.left + uavs[0].map.length * random.uniform(0.3, 0.999),
                             uavs[0].map.down + uavs[0].map.width * random.uniform(0.3, 0.999))

    # while map.is_open:
    while map.is_open:
        # if all(sub_target_reached):
        #     new_sub_targets = generate_sub_targets(map, uavs[i], visited_pts, nbr_uavs, uav_vision_range)
        #     for j in range(nbr_uavs):
        #         uavs[j].new_sub_target(new_sub_targets[j])

        for i in range(nbr_uavs):
            if uavs[i].curr_position != uavs[i].end_pos:
                other_uavs_position = {}
                for j in range(nbr_uavs):
                    if j != i:
                        other_uavs_position[j] = uavs[j].curr_position
                        print(f'uav {i} current positon: {(uavs[j].curr_position.x, uavs[j].curr_position.y)}')
                uavs[i].move(other_uavs_position)

                visited_pts.append(uavs[i].curr_position)

                # map.add_geometry(type='point', pos=uavs[i].curr_position.tuple(), size=30, color_alpha=path_colors[i], alpha=0.5)
                map.add_geometry(type='point', pos=uavs[i].curr_position.tuple(), size=size_uav, color=path_colors[i])
                map.add_geometry(type='circle', pos=uavs[i].curr_position.tuple(), radius=com_range_uav, filled=False,
                                 color=path_colors[i])

                # add the starting and goal position of each uav in the environment
                map.add_geometry(type='point', pos=starts[i].tuple(), size=size_uav, color=(100, 0, 0))
                map.add_geometry(type='point', pos=ends[i].tuple(), size=size_uav, color=(0, 100, 0))
                for j in range(len(uavs[i].final_path) - 1):
                    map.add_geometry(type='line', start=uavs[i].final_path[j].tuple(),
                                     end=uavs[i].final_path[j + 1].tuple(), color=path_colors[i])

                # add the target in the environment
                map.add_geometry(type='point', pos=target_point.tuple(), size=2*size_uav, color=(50, 50, 50))

                if target_point.dist(uavs[i].curr_position) <= uav_vision_range:
                    print('******* MISSION COMPLETE: Target Found! **********')
                    # ##
                    # ##
                    # for ii in range(nbr_uavs):
                    #     map.add_geometry(type='point', pos=uavs[ii].curr_position.tuple(), size=size_uav,
                    #                      color=path_colors[ii])
                    #     map.add_geometry(type='circle', pos=uavs[ii].curr_position.tuple(), radius=com_range_uav,
                    #                      filled=False,
                    #                      color=path_colors[ii])
                    #
                    #     # add the starting and goal position of each uav in the environment
                    #     map.add_geometry(type='point', pos=starts[ii].tuple(), size=size_uav, color=(100, 0, 0))
                    #     map.add_geometry(type='point', pos=ends[ii].tuple(), size=size_uav, color=(0, 100, 0))
                    #     for jj in range(len(uavs[ii].final_path) - 1):
                    #         map.add_geometry(type='line', start=uavs[ii].final_path[j].tuple(),
                    #                          end=uavs[ii].final_path[jj + 1].tuple(), color=path_colors[ii])
                    # ##
                    # ##
                    # display the target in different color
                    map.add_geometry(type='point', pos=target_point.tuple(), size=2*size_uav, color=(50, 0, 50))
                    # display the uav that found the target in different color
                    map.add_geometry(type='point', pos=uavs[i].curr_position.tuple(), size=size_uav, color=(50, 0, 50))
                    map.render()
                    time.sleep(30)

                #if uavs[i].curr_position.dist(uavs[i].end_pos) <= 0.2 * uav_vision_range:
                #if uavs[i].curr_position == uavs[i].end_pos:
            else:
                # sub_target_reached[i] = True
                new_sub_targets = generate_sub_targets(map, uavs[i], visited_pts, nbr_uavs, uav_vision_range)
                print('New sub targets:', new_sub_targets)
                for j in range(nbr_uavs):
                    uavs[j].new_sub_target(new_sub_targets[j])

        map.render()
        time.sleep(0.25)


"""
def main(nbr_uavs = 3):
    map = Map(480, 0, 0, 640)
    starts = [Point(0, 0), Point(0, 400), Point(320, 0)]
    ends = [Point(640, 480), Point(640, 0), Point(0, 200)]
    map.add_obstacle(CircleObstacle(pos=Point(400, 300), radius=50))
    map.add_obstacle(RectangleObstacle(top=200, down=100, left=100, right=300))
    map.add_obstacle(RectangleObstacle(top=400, down=300, left=50, right=200))
    rrtPlanners = {}

    path_colors = [(100, 0, 0), (0, 100, 0), (0, 100, 100), (100, 100, 0)]

    for i in range(nbr_uavs):
        rrtPlanners[i] = RRTStarPlanner(map, start_pos=starts[i], end_pos=ends[i], epsilon=0.2, stepSize=10)

    while map.is_open:
        for i in range(nbr_uavs):
            rrtPlanners[i].plan()
            map.add_geometry(type='point', pos=starts[i].tuple(), size=30, color=(100, 0, 0))
            map.add_geometry(type='point', pos=ends[i].tuple(), size=30, color=(0, 100, 0))
            #for node in rrtPlanners[i].nodeList:
            #    map.add_geometry(type='point', pos=node.pos.tuple())
            #    if node.parent is not None:
            #        map.add_geometry(type='line', start=node.parent.pos.tuple(), end=node.pos.tuple())
            for j in range(len(rrtPlanners[i].finalPath) - 1):
                map.add_geometry(type='line', start=rrtPlanners[i].finalPath[j].tuple(),
                                 end=rrtPlanners[i].finalPath[j + 1].tuple(), color=path_colors[i])

        map.render()
"""

if __name__ == '__main__':
    number_uavs = 5
    main(number_uavs)
