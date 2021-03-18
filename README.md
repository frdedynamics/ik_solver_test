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

