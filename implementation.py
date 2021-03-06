#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  1 09:08:43 2021

@author: prannoy
"""

from map_creator import create_main_map
import numpy as np
import cv2
import pygame

map_of_maze, o_list = create_main_map()


class NodeDetails:
    def __init__(self, point=np.array((0, 0)), previous_point=None, min_distance_from_start=np.Inf):
        self.point = point
        self.previous_point = previous_point
        self.min_distance_from_start = min_distance_from_start

    def __str__(self):
        return f"The point is {self.point}. The previous vertex is {self.previous_point}. Minimum distance from " \
               f"statrt node {self.min_distance_from_start} "


def action_set(point):
    actions_set = [np.array([1, 0]),
                   np.array([-1, 0]),
                   np.array([0, 1]),
                   np.array([0, -1]),
                   np.array([1, 1]),
                   np.array([-1, 1]),
                   np.array([1, -1]),
                   np.array([-1, -1])]

    child_states = []

    for action in actions_set:
        action = action + point

        x, y = action

        if (x < 0 or x > 299) or (y < 0 or y > 399) or map_of_maze[x, y] == 255:
            action = None

        child_states.append(action)

    return child_states


def cost_of_action(new_point, old_point):
    return np.linalg.norm(new_point - old_point)


def update_distance(node=NodeDetails(), new_distance=np.Inf, previous_point=None):
    if node.min_distance_from_start > new_distance:
        node.min_distance_from_start = new_distance
        node.previous_point = previous_point

    return node


def initial_and_goal_states():
    while True:
        x_s = int(input("Enter the x coordinate of the start node"))
        y_s = int(input("Enter the y coordinate of the start node"))
        x_g = int(input("Enter the x coordinate of the goal node"))
        y_g = int(input("Enter the y coordinate of the goal node"))

        start_point = np.array((x_s, 400 - y_s))
        goal_point = np.array((x_g, 400 - y_g))

        if (x_s < 0 or x_s > 299) or (y_s < 0 or y_s > 399) or map_of_maze[x_s, y_s] == 255:
            print("Invalid x input")
            continue

        if (x_g < 0 or x_g > 299) or (y_g < 0 or y_g > 399) or map_of_maze[x_g, y_g] == 255:
            print("Invalid y input")
            continue

        break
    return start_point, goal_point


if __name__ == '__main__':

    state_of_vertices = []
    state_of_vertices_row = []
    for i in range(0, map_of_maze.shape[0]):
        state_of_vertices_row = []
        for j in range(0, map_of_maze.shape[1]):
            state_of_vertices_row.append(NodeDetails(np.array([i, j])))

        state_of_vertices.append(state_of_vertices_row)

    del state_of_vertices_row
    del i
    del j

    s_n, g_n = initial_and_goal_states()
    start_node = NodeDetails(s_n, None, 0)
    state_of_vertices[start_node.point[0]][start_node.point[1]] = start_node
    visited_nodes = [start_node]
    open_nodes = []
    iterator = 0
    while True:

        if np.array_equal(g_n, visited_nodes[iterator].point):
            break
            print("SHORTEST PATH TO GOAL FOUND")

        possible_states = action_set(visited_nodes[iterator].point)

        for state in possible_states:
            if state is None:
                pass
            else:
                if cost_of_action(visited_nodes[iterator].point, state) + visited_nodes[
                    iterator].min_distance_from_start < \
                        state_of_vertices[state[0]][state[1]].min_distance_from_start:
                    state_of_vertices[state[0]][state[1]].min_distance_from_start = cost_of_action(
                        visited_nodes[iterator].point, state) + visited_nodes[iterator].min_distance_from_start
                    state_of_vertices[state[0]][state[1]].previous_point = visited_nodes[iterator].point
                    for node, i in zip(open_nodes, range(len(open_nodes))):
                        if np.array_equal(node.point, state_of_vertices[state[0]][state[1]].point):
                            open_nodes[i] = state_of_vertices[state[0]][state[1]]

                    open_nodes.append(state_of_vertices[state[0]][state[1]])
        next_node = NodeDetails()
        for node in open_nodes:
            if node.min_distance_from_start < next_node.min_distance_from_start:
                next_node = node

        try:
            open_nodes.remove(next_node)

        except:
            print("Error")
            print("possible Action States")
            print(possible_states)
            print(visited_nodes[iterator])
            for node in open_nodes:
                print(node)

            break

        visited_nodes.append(next_node)

        iterator += 1
        del possible_states
    # Backtracking
    visited_nodes.reverse()
    a = 0
    path_list = []
    while True:
        path_list.append(visited_nodes[a].point)

        point_to_find = visited_nodes[a].previous_point

        if point_to_find is None:
            break

        for node in visited_nodes:
            if np.array_equal(node.point, point_to_find):
                a = visited_nodes.index(node)
        
    # Visualization using Pygame
    pygame.init()

    gameDisplay = pygame.display.set_mode(map_of_maze.shape, pygame.SCALED)
    pygame.display.set_caption("Solution - Animation")

    white = (255, 255, 255)  # Background
    black = (0, 0, 0)  # Obstacle
    red = (255, 0, 0)  # Visited Node
    blue = (0, 0, 255)  # Path
    green = (0, 255, 0)  # Goal

    surface = pygame.surfarray.make_surface(map_of_maze)

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('animation.avi', fourcc, 750, (
    map_of_maze.shape[1], map_of_maze.shape[0]))  # Increase the frame rate here(set to 750), for faster video

    clock = pygame.time.Clock()

    gameDisplay.fill(white)

    done = True

    while done:
        for point in o_list:
            pygame.draw.rect(gameDisplay, black, [point[0], point[1], 1, 1])
            pygame.image.save(gameDisplay, f"/home/prannoy/s21subbmissions/661/project2/load.png")

        for node in visited_nodes:
            pygame.draw.rect(gameDisplay, red, [node.point[0], node.point[1], 1, 1])
            pygame.image.save(gameDisplay, f"/home/prannoy/s21subbmissions/661/project2/load.png")
            image = cv2.imread('load.png')
            image = cv2.flip(image, 1)
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            out.write(image)

        pygame.draw.rect(gameDisplay, green, [g_n[0], g_n[1], 2, 2])
        pygame.image.save(gameDisplay, f"/home/prannoy/s21subbmissions/661/project2/load.png")
        image = cv2.imread('load.png')
        image = cv2.flip(image, 1)
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        out.write(image)

        for point in path_list:
            pygame.draw.rect(gameDisplay, blue, [point[0], point[1], 3, 3])
            pygame.image.save(gameDisplay, f"/home/prannoy/s21subbmissions/661/project2/load.png")
            image = cv2.imread('load.png')
            image = cv2.flip(image, 1)
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            out.write(image)

        for i in range(5000):
            image = cv2.flip(image, 1)
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            image = cv2.imread('load.png')
            out.write(image)
        done = False
    pygame.quit()
