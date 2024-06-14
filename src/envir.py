import pygame
import math

class Envir:
    def __init__(self, dimensions):
        self.map_w = dimensions[0]
        self.map_h = dimensions[1]

        pygame.init()
        pygame.display.set_caption("Differential Drive Robot")
        self.map = pygame.display.set_mode((self.map_w, self.map_h))

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

    def move(self, dt, event=None):
        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_KP4:
                    self.vel_left_mps += 0.001 * self.meters_to_pixel
                elif event.key == pygame.K_KP1:
                    self.vel_left_mps -= 0.001 * self.meters_to_pixel
                elif event.key == pygame.K_KP6:
                    self.vel_right_mps += 0.001 * self.meters_to_pixel
                elif event.key == pygame.K_KP3:
                    self.vel_right_mps -= 0.001 * self.meters_to_pixel

        self.robot_x += ((self.vel_left_mps+self.vel_right_mps)/2) * math.cos(self.robot_angle)*dt
        self.robot_y -= ((self.vel_left_mps+self.vel_right_mps)/2) * math.sin(self.robot_angle)*dt
        self.robot_angle += (self.vel_right_mps - self.vel_left_mps) / self.robot_width*dt
        self.robot_rotated = pygame.transform.rotozoom(self.robot_img, math.degrees(self.robot_angle), 1)
        self.rect = self.robot_rotated.get_rect(center=(self.robot_x, self.robot_y))


if __name__ == '__main__':
    env = Envir((1200, 600))
    rob = Robot((200, 200), "img/robot.png", 0.01*3779.52) # 0.01m = 1 cm

    run = True
    dt = 0
    last_time = pygame.time.get_ticks()

    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            rob.move(dt, event)


        dt = (pygame.time.get_ticks() - last_time) / 1000
        last_time = pygame.time.get_ticks()

        pygame.display.update()
        env.map.fill(pygame.Color("white"))
        rob.move(dt)
        rob.draw(env.map)

    pygame.quit()