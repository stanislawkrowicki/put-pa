import math
import pygame
import threading
import numpy as np

import mpc
import constants
from docking_action import DockingAction

state = [0, 0, 0, 0] # [x, y, omega, v]

def run_pygame():
    global state, IS_DOCKING
    
    # Initialize Pygame
    pygame.init()

    # Screen setup
    screen_width, screen_height = 1600, 800
    screen = pygame.display.set_mode([screen_width, screen_height])
    pygame.display.set_caption("Simplified Bicycle Model")
    clock = pygame.time.Clock()

    # Colors
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    BLUE = (0, 0, 255)
    GREEN = (0, 200, 0)
    RED = (255, 0, 0)

    # Parameters
    dt = constants.T_STEP # Time step [s]
    max_steer = math.radians(30.0)  # Maximum steering angle [rad]

    target_delta = 0.0
    is_selecting_target = False
    start_mouse_x = 0
    start_mouse_y = 0

    # Main loop
    running = True
    while running:
        screen.fill(BLACK)

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                start_mouse_x, start_mouse_y = pygame.mouse.get_pos()
                is_selecting_target = True
            elif event.type == pygame.MOUSEBUTTONUP:
                if not is_selecting_target:
                    continue
                target_x = start_mouse_x / constants.RATIO
                target_y = start_mouse_y / constants.RATIO
                is_selecting_target = False
                # target_delta gets updated every tick in the main loop
                DockingAction.run_docking_thread(state[0], state[1], state[2], 
                                                 target_x, target_y, target_delta)

        # Keyboard input
        keys = pygame.key.get_pressed()

        if keys[pygame.K_SPACE]:  # Reset position
                state = [800 / constants.RATIO, 400 / constants.RATIO, 0.0, 0.0]
                steering_angle = 0.0
                acceleration = 0.0
                DockingAction.stop_docking()
                print("RESET")
        elif not DockingAction.is_docking():
            steering_angle = 0.0
            acceleration = 0.0

            if keys[pygame.K_UP]:
                acceleration = 0.8  # Accelerate
            elif keys[pygame.K_DOWN]:
                acceleration = -0.8  # Decelerate

            if keys[pygame.K_LEFT]:
                steering_angle = -max_steer  # Steer left
            elif keys[pygame.K_RIGHT]:
                steering_angle = max_steer  # Steer right

            # Update dynamics
            x, y, omega, v = state
            omega_new = omega + dt * v * math.tan(steering_angle) / constants.BICYCLE_LENGTH
            x_new = x + dt * v * math.cos(omega)
            y_new = y + dt * v * math.sin(omega)
            v_new = v + dt * acceleration
        
            # Update state
            state = [x_new, y_new, omega_new, max(0.4, v_new)]  # Prevent negative velocity

        # Draw vehicle
        def draw_vehicle(x, y, omega):
            x, y = x * constants.RATIO, y * constants.RATIO
            pygame.draw.circle(screen, GREEN, (int(x), int(y)), 10)  # Represent vehicle as a circle
            line_length = 20
            pygame.draw.line(
                screen,
                WHITE,
                (int(x), int(y)),
                (int(x + line_length * math.cos(omega)), int(y + line_length * math.sin(omega))),
                2,
            )

        if is_selecting_target:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            delta_x = mouse_x - start_mouse_x
            scaling_factor = 100
            target_delta = delta_x / scaling_factor
            target_delta = max(-math.pi, min(math.pi, target_delta))
            line_length = 50
            pygame.draw.line(
                screen,
                RED,
                (start_mouse_x, start_mouse_y),
                (start_mouse_x + line_length * math.cos(target_delta), 
                 start_mouse_y + line_length * math.sin(target_delta)),
                 2
            )

        if DockingAction.is_docking():
            state = DockingAction.get_current_state()

        draw_vehicle(state[0], state[1], state[2])

        # Update display
        pygame.display.flip()
        clock.tick(60)

    # Quit Pygame
    pygame.quit()


if __name__ == "__main__":
    state = constants.START_STATE

    # Initialize MPC
    mpc.prepare_mpc()

    run_pygame()    
