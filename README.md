How to use

1. Clone the project into a catkin workspace `git clone git@github.com:cohnt/Particle-Based-Localization.git`
2. Adjust folder paths in the following script files
..* In `single_handle.py`, change `path_to_model_file`
..* In `two_filters.py`, change `path_to_model_file`
..* In `training/detect_handles.py`, change `image_directory`
..* In `training/save_on_keypress.py`, change `image_directory`
3. `catkin_make`
4. `source devel/setup.bash`
5. Gather a training dataset using `rosrun grab_bag save_on_keypress.py`
6. Train a model using `rosrun grab_bag detect_handles.py`
7. Localize handles using `rosrun grab_bag single_handle.py` or `rosrun grab_bag two_filters.py`

---

Related Repositories

https://github.com/cohnt/ROS-Tools

https://github.com/cohnt/2D-Bag-Localization-Sim

---

Useful Commands

---

rosbag play -r 0.5 -s 7 my.bag


One of the handles is at (0.95, -0.5, 1.1) in the frame /odom


Videos

https://www.youtube.com/watch?v=8LI67jwJt6c&feature=youtu.be
https://www.youtube.com/watch?v=UOY7HeGxDeQ&feature=youtu.be
