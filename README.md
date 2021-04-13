# IKfast Test

This repository is created for test purposes (for now).



## Required:

It requires **OpenRAVE** to be installed -> http://openrave.org/docs/latest_stable/install/#install -- No PPA support for 18.04

*Building from source:* https://rospibot.azw.pt/install-openrave-ubuntu-18-04-bionic/



## Tree:

├── CMakeLists.txt
├── examples: *Modified examples from http://openrave.org/docs/latest_stable/tutorials/openravepy_examples/*
├── launch
│   ├── config
├── LICENSE.md
├── package.xml
├── README.md
├── robots: *new robots are created under here*
│   ├── planar_3dof
│   ├── sc-3dof
│   └── ur5
├── scripts: *some necessary scripts missing in installation - building from source*
│   ├── auto_create_ikfast_moveit_plugin.sh
│   ├── create_ikfast_moveit_plugin.py
│   ├── readme.txt
│   └── round_collada_numbers.py
├── src
├── urdf: *URDF/COLLADA robot models*
│   ├── iktest.cpp
│   ├── sc_3dof.dae
│   ├── sc_3dof.rounded.dae
│   ├── sc_3dof.urdf
│   ├── sc_3dof.xacro
│   ├── ur5.dae
│   └── ur5-fhdesc.urdf
└── xml: *new environments*



## Notes:

#### Venv:

Better to install Openrave in a virtual environment. Quite old libraries are being used. You don't want to mess your main Python environment.

* Create venv: `virtualenv -p /usr/bin/python venv-openrave`

* Need to activate always: `source ~/venv-openrave/bin/activate`

#### Environment Variables:

Need you add **openravepy** in your Python path (unless you define in `.bashrc` already) before `import openravepy`:

```tex
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_
export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)
```





# Gizem Local Notes:

# Installation

## OpenRAVE

http://openrave.org/docs/latest_stable/install/#install -- No PPA support for 18.04

Create venv: `virtualenv -p /usr/bin/python venv-openrave`

Need to activate always: `source ~/venv-openrave/bin/activate`

Building from source: https://rospibot.azw.pt/install-openrave-ubuntu-18-04-bionic/

### openrave_planning 

Ros package: https://github.com/jsk-ros-pkg/openrave_planning

I installed it because I need `arm_navigation_msgs` and it is cool.

Bug fixing: For those who don't have OpenRAVE installed, keep this in mind: https://github.com/roboticslab-uc3m/installation-guides/issues/65 >> https://github.com/crigroup/openrave-installation 

My solution: After git clone, `catkin_make` failed. I deleted the **openrave** folder in the clone and the rest was just fine.



**NOTES:** 

* Main tutorial for the whole process: http://docs.ros.org/en/indigo/api/moveit_ikfast/html/doc/ikfast_tutorial.html
* Installed Collada-DOM manually: https://launchpad.net/ubuntu/+source/collada-dom (in venv-openRAVE)
* `libjasper-dev` was removed from ubuntu main. Installed like that: https://stackoverflow.com/questions/44468081/unable-to-locate-package-libjasper-dev
* OpenSceneGraph-3.4 `sudo make install`looks strange:
* ![image-20210310110050090](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210310110050090.png)
* This doesn work: `openrave <myrobot_name>.dae`. I will move on anyways
* `openrave.py` and `openrave0.9.py` are located `/usr/local/bin`. When you compute `openrave.py --example X`(or `openrave0.9.py --example X`) you actually call this script. It looks important for c++ binding.

#### URDF to Collada

`rosrun collada_urdf urdf_to_collada sc_3dof.xacro sc_3dof.dae`

`rosrun collada_urdf urdf_to_collada sc_3dof.xacro sc_3dof.dae`

`rosrun moveit_ikfast round_collada_numbers.py sc_3dof.dae sc_3dof.rounded.dae 5`

### UR5e

```
xacro ur5e_robot.urdf.xacro > ur5e.urdf
rosrun collada_urdf urdf_to_collada ur5e.urdf ur5e.dae
```

- Don't forget to use `ik_solver_class_ur5e` in your node

#### Links are listed:

`~/Programs/OpenRAVE/repos/openrave/build/python$ openrave0.9-robot.py /home/gizem/catkin_ws/src/ik_solver_test/urdf/sc_3dof.dae --info links`

#### Add openravepy into python path: 

Here: 

```c
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_
export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)
source ~/venv-openrave/bin/activate
```

#### IKfast solver

@ pwd: `~/Programs/OpenRAVE/repos/openrave/python`

See the links: 

![image-20210312152014336](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210312152014336.png)

**For UR5:** ![image-20210315154125212](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210315154125212.png)

```
python ikfast.py --robot=/home/gizem/catkin_ws/src/ik_solver_test/urdf/sc_3dof.dae --baselink=1 --eelink=5 --savefile=/home/gizem/catkin_ws/src/ik_solver_test/urdf/iktest.cpp --iktype=translation3d
```

> Here the ikfast.cpp is created. Now time to create moveit plugin. But first I want to try if normal moveit ik can solve for 3d spacecraft model:

YAYY!! It doesn't work for 3D SC either :D 

Now it is time to try my new plugin:

#### Create plugin:

```
cd ~/catkin_ws/src/ik_solver_test/src

python create_ikfast_moveit_plugin.py sc_3dof manipulator sc_3dof_ikfast_manipulator_plugin Link_0 Link_EEhis /home/gizem/catkin_ws/src/ik_solver_test/generated_plugins/iktest.cpp 
```

```
Meaning:
----------
python create_ikfast_moveit_plugin.py <robot name> <planning group> <newly created empty package> <base_link> <ee_link> <path to ikfast.cpp created one step before>
```

These files are created:

![image-20210312160513613](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210312160513613.png)

# TODO:

https://groups.google.com/g/moveit-users/c/P2V9eW5BjW8 : people solve moveit KDL problem for <6DOF problem. Also there: https://github.com/ros-planning/moveit_ros/issues/257

- The Person hacked KDL library: https://www.youtube.com/watch?v=m_oqufE-5gY

- SC urdf: https://spart.readthedocs.io/en/latest/URDF_Models.html and different URFDs

- gazebo model does not appear for sc_3dof. Solution here: https://answers.ros.org/question/214712/gazebo-controller-spawner-warning/ Skipped for now

- Gazebo odom publisher does not publish tool0. It publishes wrist_3_link. The way how I moved by changing tool0 is very bad. because now the openrave takes tool0 as ee. Don't do it again. Use additional transformation from wrist3 to tool0. Issue is here: ![image-20210323234712637](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210323234712637.png)

  So, the transformation:

  ```
  rosrun tf tf_echo /wrist_3_link /tool0
  
  - Translation: [0.000, 0.082, 0.000]
  - Rotation: in Quaternion [-0.707, 0.000, 0.000, 0.707]
              in RPY (radian) [-1.571, 0.000, 0.000]
              in RPY (degree) [-90.000, 0.000, 0.000]
  
  ```

  

#### OpenRAVE addings:

http://openrave.org/docs/latest_stable/openravepy/databases.linkstatistics/



## Important

KDL limitation: https://answers.ros.org/question/152591/kdl-kinematics-solver-limitations/

Openrave IkParametrization:http://openrave.org/docs/latest_stable/coreapihtml/classOpenRAVE_1_1IkParameterization.html

Openrave Kinematic reachability: http://openrave.org/docs/0.8.2/openravepy/databases.kinematicreachability/



### Kinematic reachability

> openravepy.databases: save, saving model to /home/gizem/.openrave/robot.7edbf73fb4fc856e8294d93279d26ff2/reachability.12a06408e3f80af9e5cd98f6fe50e0ba.pp

![image-20210322195810070](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210322195810070.png)

* For ur5:

  ```
  cd ~/catkin_ws/src/ik_solver_test/robots/ur5/xml
  openrave.py --database kinematicreachability --robot=ur5.xml
  ```

  

Note: Also h5py was installed as seen.

PyQt5: http://www.minsung.org/2017/12/building-pyqt5-with-python-2-7-on-ubuntu-16-04/

	SIP install: install tar, use this after: https://askubuntu.com/questions/25961/how-do-i-install-a-tar-gz-or-tar-bz2-file

**Now try to import in a script**



## Mayavi

Mayavi is not supported for Python2 any longer: Mayavi2 instead: `sudo apt-get install mayavi2`

~~Mayavi2 looks like fine: https://github.com/scibian/mayavi2 downloaded: `~/Programs/mayavi2`~~

- ~~Cannot build.~~
- ~~installed `pip install vtk~`~

Adding system installed packages under venv: https://stackoverflow.com/questions/11441546/how-i-can-make-apt-get-install-to-my-virtualenv

> To include system site packages in your existing virtual environment open the config file: `<PATH_TO_YOUR_VENV_FOLDER>/pyvenv.cfg`
>
> and change `false` to `true` for `include-system-site-packages`
>
> ```py
> include-system-site-packages = true
> ```
>
> Save and reload your virtual environment.

Not the best option beacuse I may not want to use the current OS packages' versions if there are two. But it works for now.

Mayavi cannot run on my PC atm.



# Misc

![image-20210329143900284](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210329143900284.png)

![image-20210329143927049](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210329143927049.png)

![image-20210329144014081](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210329144014081.png)





# Externally create ik.cpp

From here: https://github.com/andyzeng/ikfastpy

At first, you must have these files available:
├── ikfast.h
├── ikfastpy.pyx
├── ikfast_wrapper.cpp
├── Kinematics.hpp
├── setup.py
└── ur5e.robot.xml

Then compute this 

```
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=ur5e.robot.xml --iktype=translation3d --baselink=3 --eelink=6 --savefile=ikfast31.cpp --maxcasedepth 1
```

Result:

├── ikfast31.cpp
is created

And finally compute `python setup.py build_ext --inplace`



**If you want to change the ik.cpp:**

> ikfast_wrapper.cpp:65:10: #include "ikfile.cpp" 



### Current available values

My workspace: for ur5: `/home/gizem/Documents/OnedriveHVL/Workspaces/PythonWS/ikfastpy/ur5_original`

![image-20210408233046614](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210408233046614.png)

​							for ur5e: `/home/gizem/Documents/OnedriveHVL/Workspaces/PythonWS/ikfastpy/ur5e`

![image-20210408232835113](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210408232835113.png)![image-20210408233425986](/home/gizem/snap/typora/33/.config/Typora/typora-user-images/image-20210408233425986.png)

```
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=ur5e.robot.xml --iktype=translation3d --baselink=3 --eelink=6 --savefile=ikfast31.cpp --maxcasedepth 1
```



**Note:** Don't forget to create ikfast61.cpp or ikfast31.cpp

**Known issues**

* ImportError: No module named ikfastpy
  * Add os.path 