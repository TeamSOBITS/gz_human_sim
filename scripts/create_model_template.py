"""
Script for adding objects in .glb or .obj format.

Note: To display in a world, you need to write it in the xacro file.
Note: This script only creates templates in the models/ directory.
Note: If you get a "model not found" error, it's usually because you haven't run colcon build.

Usage:

Navigate to the package root.

```sh
cd ~/colcon_ws/src/sobits_gazebo_world
```

Run the following script depending on the 3D model format.

```sh
python3 scripts/create_model_template.py <model-name> <obj | glb>
```

Example

```
# To use chair in .obj format
python3 scripts/create_model_template.py chair obj

# To use chair in .glb format
python3 scripts/create_model_template.py chair glb
```
"""
import os
import argparse


def create_model_directory(model_name, file_type):
    """
    Create a model directory with the necessary files and folders.

    Args:
        model_name (str): Name of the model.
        file_type (str): File type (e.g., obj, glb).
    """
    base_path = os.path.join("models", model_name)

    if os.path.exists(base_path):
        raise FileExistsError(f"Model directory '{base_path}' already exists.")

    if "models" not in os.listdir():
        raise FileNotFoundError("No 'model' directory found. Your pwd might be wrong.")

    try:
        # Create the base directory
        os.makedirs(os.path.join(base_path, "meshes"), exist_ok=False)

        # Create model.config
        config_path = os.path.join(base_path, "model.config")
        with open(config_path, "w") as config_file:
            config_file.write(f"""
<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author>
    <name>Author Name</name>
    <email>author@example.com</email>
  </author>
  <description>
    A description of the {model_name} model.
  </description>
</model>
""")

        # Create model.sdf
        sdf_path = os.path.join(base_path, "model.sdf")
        with open(sdf_path, "w") as sdf_file:
            sdf_file.write(f"""
<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{model_name}">
    <static>true</static>

    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>
    </link>

    <link name="mesh_link">
      <pose>0.0 3.0 0.7 1.5708 0 0</pose>

      <inertial>
        <mass>100.0</mass>
      </inertial>

      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{model_name}.{file_type}</uri>
          </mesh>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{model_name}.{file_type}</uri>
          </mesh>
        </geometry>

        <material>
          <diffuse>0.0 0.5 0.5 1.0</diffuse>
          <ambient>0.1 0.1 0.1 1.0</ambient>
        </material>

      </visual>
    </link>

    <joint name="base_to_mesh_joint" type="fixed">
      <parent>base_link</parent>
      <child>mesh_link</child>
    </joint>

  </model>
</sdf>
""")
        print(f"Template for '{base_path}' created successfully. Please complete the following steps:")
        print(f"1. Place {model_name}.{file_type} in 'models/{model_name}/meshes' folder (note the filename)")
        print("2. Run colcon build")
        print(f"The URI to write in xacro is <uri>model://{model_name}</uri>")

    except Exception as e:
        print(f"Error creating model directory: {e}")


def main():
    parser = argparse.ArgumentParser(description="Create a Gazebo model directory structure.")
    parser.add_argument("model_name", type=str, help="Name of the model to create.")
    parser.add_argument("file_type", type=str, choices=["obj", "glb"], help="File type for the model (e.g., obj, glb).")

    args = parser.parse_args()

    create_model_directory(args.model_name, args.file_type)


if __name__ == "__main__":
    main()
