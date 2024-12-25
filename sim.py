import math
import pygame
import mpc
import threading
import numpy as np

max_error = 0.05

RATIO = 10 # Scaling factor for the screen
START_POSITION = [800 / RATIO, 400 / RATIO, 0.0, 0.0]


X = [0, 0, 0, 0] # [x, y, omega, v]
x0 = np.array([X[0], X[1], X[2]]).reshape(-1, 1)
IS_DOCKING = False

def dock(target_x, target_y, target_delta):
    global IS_DOCKING, X
    IS_DOCKING = True

    mpc.set_simulation_target(X[0], X[1], X[2], target_x, target_y, target_delta)

    x0 = np.array([X[0], X[1], X[2]]).reshape(-1, 1)
    error = np.array([target_x, target_y, target_delta]) - x0.flatten()

    while np.abs(error[0]) > max_error or np.abs(error[1]) > max_error or np.abs(error[2]) > max_error:
        if not IS_DOCKING:
            break
        u0 = mpc.bicycle_mpc.make_step(x0)
        x0 = mpc.simulator.make_step(u0)
        x_list = x0.tolist()
        u_list = u0.tolist()
        X = [x_list[0][0], x_list[1][0], x_list[2][0], u_list[0][0]]
        error = np.array([target_x, target_y, target_delta]) - x0.flatten()
        print(error)

    IS_DOCKING = False


def run_docking_thread(target_x, target_y, target_delta):
    threading.Thread(target=dock, args=(target_x, target_y, target_delta)).start()


def run_pygame():
    global X, IS_DOCKING
    
    # Initialize Pygame
    pygame.init()

    done = False

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
    L = 2  # Length of the bicycle [m]
    dt = 0.2 # Time step [s]
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
                target_x = start_mouse_x / RATIO
                target_y = start_mouse_y / RATIO
                is_selecting_target = False
                # target_delta gets updated every tick in the main loop
                run_docking_thread(target_x, target_y, target_delta)

        # Keyboard input
        keys = pygame.key.get_pressed()

        if keys[pygame.K_SPACE]:  # Reset position
                X = [800 / RATIO, 400 / RATIO, 0.0, 0.0]
                steering_angle = 0.0
                acceleration = 0.0
                IS_DOCKING = False
                print("RESET")
        elif not IS_DOCKING:
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
            x, y, omega, v = X
            omega_new = omega + dt * v * math.tan(steering_angle) / L
            x_new = x + dt * v * math.cos(omega)
            y_new = y + dt * v * math.sin(omega)
            v_new = v + dt * acceleration
        
            # Update state
            X = [x_new, y_new, omega_new, max(0.4, v_new)]  # Prevent negative velocity

        # Draw vehicle
        def draw_vehicle(x, y, omega):
            x, y = x * RATIO, y * RATIO
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

        draw_vehicle(X[0], X[1], X[2])

        # Update display
        pygame.display.flip()
        clock.tick(60)

    # Quit Pygame
    pygame.quit()


if __name__ == "__main__":
    X = START_POSITION

    # Initialize MPC
    mpc.prepare_mpc()

    run_pygame()
