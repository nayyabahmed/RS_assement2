import turtle
wn = turtle.Screen()             # Set up the window and its attributes
wn.bgcolor("lightgreen")
dis=100

romi = turtle.Turtle()           # create romi and set his pen width
romi.pensize(5)
romi.shape("turtle")
romi.speed(1)

romi.color("hotpink")            # set his color
romi.left(90)
romi.forward(dis)                 # Let romi draw an equilateral triangle
romi.left(90)
romi.forward(dis)
romi.left(90)
romi.forward(dis)
romi.left(90)                   
romi.forward(dis)
romi.left(dis)                  # turn romi around


tess = turtle.Turtle()
tess.color("blue")
tess.shape("turtle")

dist = 5
tess.up() 
tess.goto(150,150)                    # this is new
for _ in range(30):    # start with size = 5 and grow by 2
    tess.stamp()                # leave an impression on the canvas
    tess.forward(dist)          # move tess along
    tess.right(24)              # and turn her
    dist = dist + 2
wn.exitonclick()

# for more info on turtle graphics visit below link
# https://docs.python.org/3/library/turtle.html