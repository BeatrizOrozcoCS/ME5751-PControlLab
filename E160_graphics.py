import math
from tkinter import *
from PIL import Image, ImageTk

from E160_robot import *
from cost_map import *

# Swtich these two lines to choose planner

from path_planner import * # a*
# from prm_planner import * # greddy PRM
# from Beatriz_prmplanner import * #PRM
# from prm_plannerV2 import *  # greddy PRM


class E160_graphics:
    def __init__(self, environment):
        self.environment = environment
        self.tk = Tk()
        self.tk.geometry("+300+5")
        self.scale = 250  # This number should be the same as the map image size
        self.canvas = Canvas(self.tk, width=self.environment.width * self.scale,
                             height=self.scale * self.environment.height)
        self.tk.title("Motion Planning")
        self.canvas.bind("<Button-1>", self.callback_left)
        self.canvas.pack()
        self.gui_stopped = False
        self.last_rotate_control = 0
        self.last_forward_control = 0
        self.v = 0  # linear velocity
        self.w = 0  # angular velocity

        # add motor control slider
        self.forward_control = Scale(self.tk, from_=-100, to=100, length=400, label="Forward Control cm/s",
                                     tickinterval=25, orient=HORIZONTAL)
        self.forward_control.pack(side=LEFT)

        # add rotation control slider
        self.rotate_control = Scale(self.tk, from_=-180, to=180, length=400, label="Rotate Control deg/s",
                                    tickinterval=50, orient=HORIZONTAL)
        self.rotate_control.pack(side=RIGHT)

        # add show_map/cost map button
        self.show_map_button = Button(self.tk, text="Toggle Map", anchor="s", wraplength=100, command=self.show_map)
        self.show_map_button.pack()

        # add track point button
        self.track_point_button = Button(self.tk, text="Track Point", anchor="s", wraplength=100,
                                         command=self.track_point).pack()

        # add stop button
        self.stop_button = Button(self.tk, text="Stop", anchor="s", wraplength=100, command=self.stop).pack()

        # add stop button
        self.quit_button = Button(self.tk, text="Quit", anchor="s", wraplength=100, command=self.quit).pack()

        self.left_speed = Label(self.tk, text="Max Wheel Speed (Rad/s)")
        self.left_speed.pack(side=LEFT)
        self.right_speed = Label(self.tk, text="Steering Direction (Deg)")
        self.right_speed.pack(side=RIGHT)
        self.wrong = Label(self.tk, text="impossible frame")
        self.wrong.pack()
        self.impossible = 0

        self.arrow_tkimgs = []
        self.arrow_images = []
        self.arrow_points = []

        self.current_map_show = 0  # currently is showing map: 0 showing costmap: 1
        self.map = cost_map(self)

        # Un-comment the following two lines for lab 3 and lab 4
        self.path = path_planner(self)
        self.canvas.bind("<Button-3>", self.callback_right)

        # try:
        #   self.map = cost_map(self) 
        # except:
        #   self.show_map_button.configure(state="disabled")
        # draw first robot

        for r in self.environment.robots:
            self.initial_draw_robot(r)

    def draw_wall(self, wall):

        wall_points = self.scale_points(wall.points, self.scale)
        wall.poly = self.canvas.create_polygon(wall_points, fill='black')

    # convert a point in world frame to canvas frame (with 0,0 on corner) 
    def scale_points(self, points, scale):
        scaled_points = []
        for i in range(len(points) - 1):

            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                scaled_points.append(self.environment.width / 2 * scale + points[i])

                # only flip y for x,y points, not for circle radii
                scaled_points.append(self.environment.height / 2 * scale - points[i + 1])

        return scaled_points

    def reverse_scale_points(self, points, scale):
        reverse_scaled_points = []
        for i in range(len(points) - 1):
            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                reverse_scaled_points.append(-self.environment.width / 2 * scale + points[i])

                # only flip y for x,y points, not for circle radii
                reverse_scaled_points.append(self.environment.height / 2 * scale - points[i + 1])

        return reverse_scaled_points

    def initial_draw_robot(self, robot):
        # open image
        robot.robot_gif = Image.open("truck.png").convert('RGBA')
        robot.robot_gif = robot.robot_gif.resize((robot.L_pixel, robot.L_pixel), Image.ANTIALIAS)
        self.arrow_png = Image.open("arrows.png").convert('RGBA')
        # im = cv2.imread("maps/testmap.png",cv2.IMREAD_GRAYSCALE)
        # im.resize(self.environment.width*self.scale,self.environment.height*self.scale)
        # print (im.shape)

    def draw_map(self, map_img):
        # self.map_img = Image.open(map)
        # self.map_img = self.map_img.resize((width,height),Image.ANTIALIAS)
        self.map_img = map_img
        self.canvas.mapCanvasImg = ImageTk.PhotoImage(self.map_img)
        self.canvas.create_image(0, 0, anchor='nw', image=self.canvas.mapCanvasImg)

    def draw_path(self, path_img):
        self.path_img = path_img
        self.canvas.pathCanvasImg = ImageTk.PhotoImage(self.path_img)
        self.canvas.create_image(0, 0, anchor='nw', image=self.canvas.pathCanvasImg)

    def draw_robot(self, robot):
        # gif update
        robot.tkimage = ImageTk.PhotoImage(robot.robot_gif.rotate(180 / 3.14 * robot.state.theta))
        robot.image = self.canvas.create_image(robot.state.x, robot.state.y, image=robot.tkimage)
        robot_points = self.scale_points([robot.state.x, robot.state.y], self.scale)
        self.canvas.coords(robot.image, *robot_points)

    def set_velocity_disp(self, robot):
        self.left_speed.config(text="Max Wheel Speed " + str(robot.state.phi_l))
        self.right_speed.config(text="Steering Direction " + str(robot.state.phi_r))

        if (abs(robot.state.phi_l) > 20 or abs(robot.state.phi_r) > 40):
            self.impossible += 1

        self.wrong.config(text="impossible frame " + str(self.impossible))

    def draw_waypoints(self, robot):
        self.arrow_tkimgs = []
        self.arrow_images = []
        self.arrow_points = []
        if self.environment.control_mode == "AUTONOMOUS CONTROL MODE":
            for i in range(robot.state_des.get_des_size()):
                self.arrow_tkimgs.append(
                    ImageTk.PhotoImage(self.arrow_png.rotate(180 / 3.1415 * robot.state_des.theta[i])))
                self.arrow_images.append(
                    self.canvas.create_image(robot.state_des.x[i], robot.state_des.y[i], image=self.arrow_tkimgs[i]))
                self.arrow_points.append(self.scale_points([robot.state_des.x[i], robot.state_des.y[i]], self.scale))
                self.canvas.coords(self.arrow_images[i], *self.arrow_points[i])

    def get_inputs(self):
        pass

    def show_map(self):
        if self.current_map_show == 0:
            self.map.show_vis_map()
            self.current_map_show = 1

        else:
            self.map.show_map()
            self.current_map_show = 0

    def track_point(self):
        self.environment.control_mode = "AUTONOMOUS CONTROL MODE"

        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0
        self.v = 0
        self.w = 0

    def stop(self):
        self.environment.control_mode = "MANUAL CONTROL MODE"

        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0
        self.v = 0
        self.w = 0

    def quit(self):
        self.environment.control_mode = "MANUAL CONTROL MODE"
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.gui_stopped = True

    def callback_left(self, event):
        desired_points = self.reverse_scale_points([float(event.x), float(event.y)], self.scale)
        robot = self.environment.robots[0]
        robot.state_des.reset_destination(desired_points[0], desired_points[1], 0)
        print("New clicked robot travel target", robot.state_des.x, robot.state_des.y)

    def callback_right(self, event):
        desired_points = self.reverse_scale_points([float(event.x), float(event.y)], self.scale)
        print("New clicked goal position", desired_points[0], desired_points[1])
        self.path.set_goal(world_x=desired_points[0], world_y=desired_points[1], world_theta=.0)

        self.path.plan_path()
        self.path._show_path()

    def send_robot_commands(self):
        # # check to see if forward slider has changed
        # if abs(self.forward_control.get()-self.last_forward_control) > 0:
        #     self.rotate_control.set(0)       
        #     self.last_forward_control = self.forward_control.get()
        #     self.last_rotate_control = 0         
        #     self.environment.control_mode = "MANUAL CONTROL MODE"

        #     # extract what the R and L motor signals should be
        #     # self.R = self.forward_control.get()
        #     self.L = self.forward_control.get()

        # # check to see if rotate slider has changed
        # elif abs(self.rotate_control.get()-self.last_rotate_control) > 0:
        #     self.forward_control.set(0)       
        #     self.last_rotate_control = self.rotate_control.get()
        #     self.last_forward_control = 0         
        #     self.environment.control_mode = "MANUAL CONTROL MODE"

        #     # extract what the R and L motor signals should be
        #     self.R = -self.rotate_control.get()
        # self.L = self.rotate_control.get()

        # if manual mode, set motors
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            self.v = self.forward_control.get()  # get linear velocity
            self.w = self.rotate_control.get()  # get angular velocity

            # tell robot what the values should be
            robot = self.environment.robots[0]
            robot.set_motor_control(self.v, self.w / 180.0 * math.pi)  # in E160_robot.py

    # called at every iteration of main loop
    def update(self):
        # draw robots
        for r in self.environment.robots:
            self.draw_robot(r)
            self.set_velocity_disp(r)
            self.draw_waypoints(r)

            # draw particles

        # draw sensors

        # update the graphics
        self.tk.update()

        # check for gui buttons
        self.get_inputs()

        # send commands to robots
        self.send_robot_commands()

        # check for quit
        if self.gui_stopped:
            self.environment.quit()
            return False
        else:
            return True
