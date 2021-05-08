import pygame
import serial
import time


pygame.init()
screen = pygame.display.set_mode((10, 10))

ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
ser.flush()

done = False
line = "mot" + str(90) + "\n"

power = 20

print("go")
while not done:
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                line = "mot" + str(90 +power) 
            elif event.key == pygame.K_DOWN:
                line = "mot" + str(90 - power) 
            
            if event.key == pygame.K_RIGHT:
                line = "ser" + str(50) 
            elif event.key == pygame.K_LEFT:
                line = "ser" + str(130) 

            if event.key == pygame.K_p:
                power +=5
                print("the power is now: ", power)
            elif event.key == pygame.K_m:
                power -=5
                print("the power is now: ", power)

            
            elif event.key == pygame.K_q:
                done = True

        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                line = "mot" + str(90) 
            elif event.key == pygame.K_DOWN:
                line = "mot" + str(90) 
            
            if event.key == pygame.K_RIGHT:
                line = "ser" + str(90) 
            elif event.key == pygame.K_LEFT:
                line = "ser" + str(90) 
        else:
            continue
        time.sleep(0.03)
        line = line + "\n"
        ser.write(line.encode())
        print(line)
    rep = ser.read_all()
    if(rep.decode() !=''):
        print(rep)
                
