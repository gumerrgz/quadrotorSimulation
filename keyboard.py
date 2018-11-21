#Gumer Rodriguez
#Program to print the keyboard key number in pygame

import pygame, sys
from pygame.locals import *

pygame.init()
pygame.display.set_mode((100,100))

while True:
   for event in pygame.event.get():
      if event.type == QUIT: sys.exit()
      if event.type == KEYDOWN and event.dict['key'] == 50:
         print 'break'
   pygame.event.pump()