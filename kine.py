import numpy as np
import pygame
from ctypes import CDLL, c_double

# Load shared library
tmotor = CDLL('./tmotor.so')

# Inverse kinematics function
def inverse_kinematics(x, y, degree, L1, L2, L3):
    tmp1 = x - L3 * np.cos(degree)
    tmp2 = y - L3 * np.sin(degree)
    d = np.sqrt(tmp1**2 + tmp2**2)
    cos_theta_2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta_2 = np.clip(cos_theta_2, -1.0, 1.0)
    theta_2 = np.arccos(cos_theta_2)
    k1 = L1 + L2 * np.cos(theta_2)
    k2 = L2 * np.sin(theta_2)
    theta_1 = np.arctan2(tmp2, tmp1) - np.arctan2(k2, k1)
    theta_3 = degree - theta_1 - theta_2
    return theta_1, theta_2, theta_3

# Function to plot the arm
def plot_arm(x, y, degree, L1, L2, L3, angles, screen, screen_size, scale):
    theta_1, theta_2, theta_3 = angles
    x0, y0 = screen_size[0] // 2, screen_size[1] // 2
    x1 = x0 + scale * L1 * np.cos(theta_1)
    y1 = y0 - scale * L1 * np.sin(theta_1)
    x2 = x1 + scale * L2 * np.cos(theta_1 + theta_2)
    y2 = y1 - scale * L2 * np.sin(theta_1 + theta_2)
    x3 = x2 + scale * L3 * np.cos(degree)
    y3 = y2 - scale * L3 * np.sin(degree)

    screen.fill((255, 255, 255))
    pygame.draw.line(screen, (255, 0, 0), (x0, y0), (x1, y1), 5)
    pygame.draw.line(screen, (0, 255, 0), (x1, y1), (x2, y2), 5)
    pygame.draw.line(screen, (0, 0, 255), (x2, y2), (x3, y3), 5)
    pygame.draw.circle(screen, (0, 0, 0), (int(x0 + scale * x), int(y0 - scale * y)), 5)

    font = pygame.font.SysFont('Arial', 25)
    text = font.render(f'Theta 1: {np.degrees(theta_1):.2f}°', True, (0, 0, 0))
    screen.blit(text, (20, 20))
    text = font.render(f'Theta 2: {np.degrees(theta_2):.2f}°', True, (0, 0, 0))
    screen.blit(text, (20, 50))
    text = font.render(f'Theta 3: {np.degrees(theta_3):.2f}°', True, (0, 0, 0))
    screen.blit(text, (20, 80))

    pygame.display.flip()

# Function to send positions to the motors
def send_to_motors(angles):
    theta_1, theta_2, theta_3 = angles
    positions = [np.degrees(theta_1), np.degrees(theta_2), np.degrees(theta_3)]
    for i, position in enumerate(positions):
        tmotor.sendPosition(c_double(position), i + 12)

# Pygame GUI for capturing clicks
def run_gui(L1, L2, L3):
    pygame.init()
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    screen_size = pygame.display.get_surface().get_size()
    pygame.display.set_caption("Set Arm Position")

    max_length = L1 + L2 + L3
    scale = min(screen_size[0] / (2 * max_length), screen_size[1] / (2 * max_length))

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                x, y = pygame.mouse.get_pos()
                y = screen_size[1] - y
                degree = 0
                angles = inverse_kinematics((x - screen_size[0] // 2) / scale, (y - screen_size[1] // 2) / scale, degree, L1, L2, L3)
                send_to_motors(angles)
                plotI understand. Let's complete the Python code to ensure it integrates with the provided C++ code, allowing control of motors 12, 13, and 14 using inverse kinematics through Pygame. 

Here’s how you can proceed:

### Python Code to Control Motors using Inverse Kinematics:

Ensure you have compiled the `tmotor.so` shared library from the C++ code to enable Python to call C++ functions.

1. **Python Code for Pygame Interface and Inverse Kinematics:**

```python
import numpy as np
import pygame
from ctypes import CDLL, c_double

# Load shared library
tmotor = CDLL('./tmotor.so')

# Inverse kinematics function
def inverse_kinematics(x, y, degree, L1, L2, L3):
    tmp1 = x - L3 * np.cos(degree)
    tmp2 = y - L3 * np.sin(degree)
    d = np.sqrt(tmp1**2 + tmp2**2)
    cos_theta_2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta_2 = np.clip(cos_theta_2, -1.0, 1.0)
    theta_2 = np.arccos(cos_theta_2)
    k1 = L1 + L2 * np.cos(theta_2)
    k2 = L2 * np.sin(theta_2)
    theta_1 = np.arctan2(tmp2, tmp1) - np.arctan2(k2, k1)
    theta_3 = degree - theta_1 - theta_2
    return theta_1, theta_2, theta_3

# Function to plot the arm
def plot_arm(x, y, degree, L1, L2, L3, angles, screen, screen_size, scale):
    theta_1, theta_2, theta_3 = angles
    x0, y0 = screen_size[0] // 2, screen_size[1] // 2
    x1 = x0 + scale * L1 * np.cos(theta_1)
    y1 = y0 - scale * L1 * np.sin(theta_1)
    x2 = x1 + scale * L2 * np.cos(theta_1 + theta_2)
    y2 = y1 - scale * L2 * np.sin(theta_1 + theta_2)
    x3 = x2 + scale * L3 * np.cos(degree)
    y3 = y2 - scale * L3 * np.sin(degree)

    screen.fill((255, 255, 255))
    pygame.draw.line(screen, (255, 0, 0), (x0, y0), (x1, y1), 5)
    pygame.draw.line(screen, (0, 255, 0), (x1, y1), (x2, y2), 5)
    pygame.draw.line(screen, (0, 0, 255), (x2, y2), (x3, y3), 5)
    pygame.draw.circle(screen, (0, 0, 0), (int(x0 + scale * x), int(y0 - scale * y)), 5)

    font = pygame.font.SysFont('Arial', 25)
    text = font.render(f'Theta 1: {np.degrees(theta_1):.2f}°', True, (0, 0, 0))
    screen.blit(text, (20, 20))
    text = font.render(f'Theta 2: {np.degrees(theta_2):.2f}°', True, (0, 0, 0))
    screen.blit(text, (20, 50))
    text = font.render(f'Theta 3: {np.degrees(theta_3)::.2f}°', True, (0, 0, 0))
    screen.blit(text, (20, 80))

    pygame.display.flip()

# Function to send positions to the motors
def send_to_motors(angles):
    theta_1, theta_2, theta_3 = angles
    positions = [np.degrees(theta_1), np.degrees(theta_2), np.degrees(theta_3)]
    for i, position in enumerate(positions):
        tmotor.sendPosition(c_double(position), c_double(i + 12))

# Pygame GUI for capturing clicks
def run_gui(L1, L2, L3):
    pygame.init()
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    screen_size = pygame.display.get_surface().get_size()
    pygame.display.set_caption("Set Arm Position")

    max_length = L1 + L2 + L3
    scale = min(screen_size[0] / (2 * max_length), screen_size[1] / (2 * max_length))

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                x, y = pygame.mouse.get_pos()
                y = screen_size[1] - y
                degree = 0
                angles = inverse_kinematics((x - screen_size[0] // 2) / scale, (y - screen_size[1] // 2) / scale, degree, L1, L2, L3)
                send_to_motors(angles)
                plot_arm((x - screen_size[0] // 2) / scale, (y - screen_size[1] // 2) / scale, degree, L1, L2, L3, angles, screen, screen_size, scale)
    pygame.quit()

# Main function
if __name__ == "__main__":
    L1, L2, L3 = 515.0, 350.0, 320.0  # Lengths of arm segments
    run_gui(L1, L2, L3)
