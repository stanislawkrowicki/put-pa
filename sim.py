import math
import pygame

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
L = 9  # Length of the bicycle [m]
dt = 0.1  # Time step [s]
ratio = 10  # Scaling factor for visualization
max_steer = math.radians(30.0)  # Maximum steering angle [rad]

# Initial state
X = [800 / ratio, 400 / ratio, 0.0, 0.4]  # [x, y, omega, v]
# Main loop
running = True
while running:
    screen.fill(BLACK)

    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Keyboard input
    keys = pygame.key.get_pressed()
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

    if keys[pygame.K_SPACE]:  # Reset position
        X = [800 / ratio, 400 / ratio, 0.0, 0.0]

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
        x, y = x * ratio, y * ratio
        pygame.draw.circle(screen, GREEN, (int(x), int(y)), 10)  # Represent vehicle as a circle
        line_length = 20
        pygame.draw.line(
            screen,
            WHITE,
            (int(x), int(y)),
            (int(x + line_length * math.cos(omega)), int(y + line_length * math.sin(omega))),
            2,
        )

    draw_vehicle(X[0], X[1], X[2])

    # Update display
    pygame.display.flip()
    clock.tick(60)

# Quit Pygame
pygame.quit()