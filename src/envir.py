import pygame
import math
import numpy as np

class Envir:
    def __init__(self, dimensions):
        self.map_w = dimensions[0]
        self.map_h = dimensions[1]

        # Handle initialization of pygame
        pygame.init()
        pygame.display.set_caption("Differential Drive Robot")
        self.map = pygame.display.set_mode((self.map_w, self.map_h))

        # Text variables
        self.font = pygame.font.Font('freesansbold.ttf', 50)
        self.text = self.font.render('default', True, pygame.Color("black"))
        self.textRect = self.text.get_rect()
        self.textRect.center = (dimensions[0] - 600, dimensions[1] - 50)
        self.trail_set = []

    def display_info(self, vel_l, vel_r, thetha):
        display_text =  f"Vel l: {vel_l}, Vel R: {vel_r}, theta: {int(math.degrees(thetha))}"
        self.text = self.font.render(display_text, True, pygame.Color("white"),
                                     pygame.Color("black"))
        self.map.blit(self.text, self.textRect)

    def trail(self, pos):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(self.map, pygame.Color("blue"),
                             (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]))
            
        if self.trail_set.__sizeof__() > 30000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)

    def robot_frame(self, pos, rotation):
        n = 80
        center_x, center_y = pos
        x_axis = (center_x + n*math.cos(-rotation), center_y + n*math.sin(-rotation))
        y_axis = (center_x + n*math.cos(-rotation + math.pi/2),
                  center_y + n*math.sin(-rotation + math.pi/2))
        pygame.draw.line(self.map, pygame.Color("red"), (center_x, center_y), x_axis, 3)
        pygame.draw.line(self.map, pygame.Color("green"), (center_x, center_y), y_axis, 3)
        #pygame.draw.circle(self.map, pygame.Color("green"), center=x_axis, radius=20)

    def draw_goal(self, pos):
        pygame.draw.circle(self.map, pygame.Color("green"), center=pos, radius=20)



class Robot:
    def __init__(self, start_pos, robot_img, robot_width):
        # Constants
        self.meters_to_pixel = 3779.52
        # Robot dimensions
        self.robot_width = robot_width
        self.robot_x = start_pos[0]
        self.robot_y = start_pos[1]
        # Robot speed/direction
        self.robot_angle = 0
        self.vel_left_mps = 0 # 0.01 * self.meters_to_pixel # meters/s
        self.vel_right_mps = 0 # 0.01 * self.meters_to_pixel # meters/s
        self.max_speed = 0.02 * self.meters_to_pixel
        self.min_speed = -0.02 * self.meters_to_pixel
        # Graphics
        self.robot_img = pygame.image.load(robot_img)
        self.robot_rotated = self.robot_img
        self.rect = self.robot_rotated.get_rect(center=(self.robot_x, self.robot_y))


    def draw(self, map):
        map.blit(self.robot_rotated, self.rect)

    def update_velocity(self, event):
        if event.key == pygame.K_KP4:
            self.vel_left_mps += 0.001 * self.meters_to_pixel
        elif event.key == pygame.K_KP1:
            self.vel_left_mps -= 0.001 * self.meters_to_pixel
        elif event.key == pygame.K_KP6:
            self.vel_right_mps += 0.001 * self.meters_to_pixel
        elif event.key == pygame.K_KP3:
            self.vel_right_mps -= 0.001 * self.meters_to_pixel

    def update_position(self, dt):
        self.robot_x += ((self.vel_left_mps + self.vel_right_mps)/2) * math.cos(self.robot_angle)*dt
        self.robot_y -= ((self.vel_left_mps + self.vel_right_mps)/2) * math.sin(self.robot_angle)*dt
        self.robot_angle += (self.vel_right_mps - self.vel_left_mps) / self.robot_width*dt
        # Reset angle when one lap is completed
        if self.robot_angle > 2*math.pi or self.robot_angle < - 2*math.pi:
            self.robot_angle = 0

        # Dont allow speed to go over max speed
        self.vel_left_mps = max(self.vel_left_mps, self.min_speed)
        self.vel_right_mps = max(self.vel_right_mps, self.min_speed)
        # Dont allow speed to go below min speed
        self.vel_left_mps = min(self.vel_left_mps, self.max_speed)
        self.vel_right_mps = min(self.vel_right_mps, self.max_speed)

        # Rotate image
        self.robot_rotated = pygame.transform.rotozoom(self.robot_img, math.degrees(self.robot_angle), 1)
        self.rect = self.robot_rotated.get_rect(center=(self.robot_x, self.robot_y))

    def follow_goal(self, goal_coordinates):
        n = 80
        goal_xy = [goal_coordinates[0] - self.robot_x, goal_coordinates[1] - self.robot_y]
        x_axis = [n*math.cos(-self.robot_angle), n*math.sin(-self.robot_angle)]
        # print(x_axis, goal_xy)

        px = (goal_coordinates[0] - self.robot_x)**2
        py = (goal_coordinates[1] - self.robot_y)**2

        dist = math.sqrt(px + py)
        if dist < 30:
            self.vel_left_mps = 0
            self.vel_right_mps = 0
            return


        unit_vector1 = goal_xy / np.linalg.norm(goal_xy)
        unit_vector2 = x_axis / np.linalg.norm(x_axis)

        minor = np.linalg.det(np.stack((unit_vector1[-2:], unit_vector2[-2:])))
        if minor == 0:
            raise NotImplementedError('Too odd vectors =(')
        angle = -np.sign(minor) * np.arccos(np.clip(np.dot(unit_vector1, unit_vector2), -1.0, 1.0))
        angle = math.degrees(angle)
        
        if angle > 3:
            self.vel_left_mps += 0.005 * self.meters_to_pixel
            self.vel_right_mps = 0
        elif angle < -3:
            self.vel_right_mps += 0.005 * self.meters_to_pixel
            self.vel_left_mps = 0
        else:
            self.vel_right_mps += 0.005 * self.meters_to_pixel
            self.vel_left_mps += 0.005 * self.meters_to_pixel





if __name__ == '__main__':
    env = Envir((1200, 600))
    rob = Robot((200, 200), "img/robot.png", 0.021*3779.52) # 0.01m = 1 cm

    run = True
    dt = 0
    last_time = pygame.time.get_ticks()

    # Goal variables
    goal_flag = False
    goal_pos = (0, 0)


    while run:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                goal_flag = True
                goal_pos = pygame.mouse.get_pos()
            if event.type == pygame.KEYDOWN:
                rob.update_velocity(event)


        # Handle time
        dt = (pygame.time.get_ticks() - last_time) / 1000
        last_time = pygame.time.get_ticks()

        # Fill background with white
        env.map.fill(pygame.Color("white"))

        # Draw goal
        if goal_flag:
            env.draw_goal(goal_pos)
            rob.follow_goal(goal_pos)

        # Handle display
        rob.update_position(dt)
        env.display_info(int(rob.vel_left_mps), int(rob.vel_right_mps), rob.robot_angle)
        rob.draw(env.map)
        env.robot_frame((rob.robot_x, rob.robot_y), rob.robot_angle)
        env.trail((rob.robot_x, rob.robot_y))

        # Update display
        pygame.display.update()

    pygame.quit()