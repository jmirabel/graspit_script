# GraspIt Script

This repo allows to generate grasp poses for an object and a gripper using GraspIt.

## Setup

Install GraspIt and this repo in the same installation environment.
To setup the installation environnment, you can use the following bash script.
```bash
env_dir=/home/gepetto/graspit_install
export LD_LIBRARY_PATH=${env_dir}/lib:${LD_LIBRARY_PATH}
export PATH=${env_dir}/bin:${PATH}
export CMAKE_PREFIX_PATH=${env_dir}:${CMAKE_PREFIX_PATH}
export GRASPIT=${env_dir}/graspit_data
export GRASPIT_PLUGIN_DIR=${env_dir}/lib/plugins
```

## Usage

You have to first generate a GraspIt world. Either create your own world using `graspit_simulator` GUI
or use the one from this repo. To use the one from this repo, you first have to copy the content of `data` into
`$GRASPIT` directory.
```bash
graspit_simulator -w tless/obj_21 --headless -p graspit_script
```

## TODO

- Make the parameters of the algo customizable somehow.
- Export the generated grasps in a parsable format.
