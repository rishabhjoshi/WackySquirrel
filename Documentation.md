#Code Documentation::

##Motion:

###1. Forward and Backward motion:

  a> forward() and backward()
  
  b> velocity(<speed of left motor(0-255)>,<speed of right motor(0,255)>)
  
  c> _delay_ms(time in millisec)
  
  d> stop()

####OR
  
  a>forward_mm(dist. in mm),backward_mm(dist.in mm)

###2. Left and right Turning :
  
a> left() and right()

  b>velocity(lwheel,rwheel) >>>> keep the values very low

  c>stop()

  ####OR

  a>left_degree(angle in degrees) , right_degree(angle in degrees)  >>> this method id not that accurate



##Timer:

_delay_ms(delay in milliseconds)



##Color Detection:


color_detect()-- return char value

char color - stores the value of colour

##Sharp Sensors:

###vars:

sharp_left, sharp_right, sharp_front   >>> values stored in mm

sharp_left_diff, sharp_right_diff, sharp_front_diff  >>> difference betweenprevious and current values

print_sharp_sensor()--- prints the value of sharp sensor on LCD and puts value in the above variables


##Line Sensor: 

print_line_sensor()>>> prints the value line sensor on lcd, initaialises the values valiables-- left_line, right_line, centre_line

##Print LCD: 

Lcd_cursor_char_print(row,column,letter)

##Servo motors:

Sero_1(angles in degrees)

wait()

servo_1_free()


