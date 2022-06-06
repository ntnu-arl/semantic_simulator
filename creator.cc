/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ignition/msgs/entity_factory.pb.h>
#include <iostream>
#include <string>
#include <ignition/transport/Node.hh>
#include <cstdlib>
#include <bits/stdc++.h>
#include <chrono>
#include <thread>
#include <list>
// Create a transport node.
ignition::transport::Node node;

// timeout used for services
constexpr unsigned int timeout = 5000;


void createLight()
{
  bool result;
  ignition::msgs::Boolean rep;
//! [create light]
  ignition::msgs::EntityFactory entityFactoryRequest;

  entityFactoryRequest.mutable_light()->set_name("point");
  entityFactoryRequest.mutable_light()->set_range(4);
  entityFactoryRequest.mutable_light()->set_attenuation_linear(0.5);
  entityFactoryRequest.mutable_light()->set_attenuation_constant(0.2);
  entityFactoryRequest.mutable_light()->set_attenuation_quadratic(0.01);
  entityFactoryRequest.mutable_light()->set_cast_shadows(false);
  entityFactoryRequest.mutable_light()->set_type(ignition::msgs::Light::POINT);
  ignition::msgs::Set(
    entityFactoryRequest.mutable_light()->mutable_direction(),
    ignition::math::Vector3d(0.5, 0.2, -0.9));
  ignition::msgs::Set(entityFactoryRequest.mutable_light()->mutable_pose(),
    ignition::math::Pose3d(0.0, 0, 3.0, 0.0, 0.0, 0.0));
//! [create light]

//! [call service create]
  bool executedEntityFactory = node.Request("/world/empty/create",
        entityFactoryRequest, timeout, rep, result);
  if (executedEntityFactory)
  {
    if (result)
      std::cout << "Light was created : [" << rep.data() << "]" << std::endl;
    else
    {
      std::cout << "Service call failed" << std::endl;
      return;
    }
  }
  else
    std::cerr << "Service call timed out" << std::endl;
//! [call service create]
}

void createEntityFromStr(const std::string modelStr, const std::string name)
{
//! [call service create sphere]
  bool result;
  ignition::msgs::EntityFactory req;

  ignition::msgs::Boolean res;
  req.set_sdf(modelStr);
  req.set_name(name);
  //req.set_allow_renaming(1);

  bool executed = node.Request("/world/empty/create",
            req, timeout, res, result);
  if (executed)
  {
    if (result)
      std::cout << "Entity was created : [" << res.data() << "]" << std::endl;
    else
    {
      std::cout << "Service call failed" << std::endl;
      return;
    }
  }
  else
    std::cerr << "Service call timed out" << std::endl;
//! [call service create sphere]
}

void removeEntityFromStr(const std::string name, ignition::msgs::Entity_Type ts)
{
//! [call service create sphere]
  bool result;
  ignition::msgs::Entity req;
  ignition::msgs::Boolean res;

  // ignition::msgs::Entity_Type entity_type = 2; //MODEL
  req.set_name(name);
  req.set_type(ts);

  bool executed = node.Request("/world/empty/remove",
            req, timeout, res, result);
  if (executed)
  {
    if (result)
      std::cout << "Entity was removed : [" << res.data() << "]" << std::endl;
    else
    {
      std::cout << "Service call failed" << std::endl;
      return;
    }
  }
  else
    std::cerr << "Service call timed out" << std::endl;
//! [call service create sphere]
}
//////////////////////////////////////////////////
std::string generateLightStr(
  const std::string light_type, const std::string name,
  const bool cast_shadows, const ignition::math::Pose3d pose,
  const ignition::math::Color diffuse,
  const ignition::math::Color specular,
  const double attRange, const double attConstant,
  const double attLinear, const double attQuadratic,
  const ignition::math::Vector3d direction,
  const double spot_inner_angle,
  const double spot_outer_angle,
  const double spot_falloff
)
{
//! [create light str]
  std::string lightStr = std::string("<sdf version='1.7'>") +
    "<light type='" + light_type + "' name='" + name + "'> " +
      "<cast_shadows>" + std::to_string(cast_shadows) + "</cast_shadows>" +
      "<pose>" +
      std::to_string(pose.Pos().X()) + " " +
      std::to_string(pose.Pos().Y()) + " " +
      std::to_string(pose.Pos().Z()) + " " +
      std::to_string(pose.Rot().Roll()) + " " +
      std::to_string(pose.Rot().Pitch()) + " " +
      std::to_string(pose.Rot().Yaw()) +
      "</pose>" +
      "<diffuse>" +
      std::to_string(diffuse.R()) + " " +
      std::to_string(diffuse.G()) + " " +
      std::to_string(diffuse.B()) + " " +
      std::to_string(diffuse.A()) +
      "</diffuse>" +
      "<specular>" +
      std::to_string(specular.R()) + " " +
      std::to_string(specular.G()) + " " +
      std::to_string(specular.B()) + " " +
      std::to_string(specular.A()) +
      "</specular>" +
      "<attenuation>" +
        "<range>" + std::to_string(attRange) + "</range>" +
        "<constant>" + std::to_string(attConstant) + "</constant>" +
        "<linear>" + std::to_string(attLinear) +   "</linear>" +
        "<quadratic>" + std::to_string(attQuadratic) + "</quadratic>" +
      "</attenuation>" +
      "<direction>" +
      std::to_string(direction.X()) + " " +
      std::to_string(direction.Y()) + " " +
      std::to_string(direction.Z()) +
      "</direction>" +
      "<spot>" +
        "<inner_angle>" + std::to_string(spot_inner_angle) + "</inner_angle>" +
        "<outer_angle>" + std::to_string(spot_outer_angle) + "</outer_angle>" +
        "<falloff>" + std::to_string(spot_falloff) + "</falloff>" +
      "</spot>" +
    "</light></sdf>";
//! [create light str]
  return lightStr;
}

std::string random_num(int min, int max){
  return std::to_string(min + (rand() % static_cast<int>(max - min + 1)));
}

int random_num_int(int min, int max){
  return min + (rand() % static_cast<int>(max - min + 1));
}

std::list<int> randomize_pose(bool returnint){
  int max = 30;
  int min = -30;
  std::list<int> integer_list;
  int z = random_num_int(0,5);
  integer_list.push_back(z);
  for (int i=0;i < 2;i++){
    int output = min + (rand() % static_cast<int>(max - min + 1));
    integer_list.push_front(output);
  }
  if (returnint){
    return integer_list;
  } 
}

std::list<int> randomize_pose_z(bool returnint){
  int max = 30;
  int min = -30;
  std::list<int> integer_list;
  int z = 0;
  integer_list.push_back(z);
  for (int i=0;i < 2;i++){
    int output = min + (rand() % static_cast<int>(max - min + 1));
    integer_list.push_front(output);
  }
  if (returnint){
    return integer_list;
  } 
}



std::string deviate_poses(std::list<int> integer_list){
  std::list<int> new_list;
  std::string pose_string = "<pose>";
  for (auto v : integer_list)
      new_list.push_back(v + random_num_int(0,0));
  for (auto v : new_list)
      pose_string += std::to_string(v) + " " ;
  pose_string += "0 0 0</pose>";
  return pose_string;
}


void create_list_of_poses(){
  std::list<int> pose = randomize_pose(1);
  for (auto v : pose)
    std::cout << "pose" << v << std::endl;
}

void create_random_entity(std::string name, std::string pose , std::string geometry){
  removeEntityFromStr(name, ignition::msgs::Entity_Type_MODEL);
  removeEntityFromStr(name, ignition::msgs::Entity_Type_MODEL);
  removeEntityFromStr(name, ignition::msgs::Entity_Type_VISUAL);

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  std::string intro_string = R"(
    <?xml version="1.0" ?>
    <sdf version='1.7'>)";
  
  intro_string += R"(
      <model name=')" + name + "'>" + R"(
        <link name='link'>
        )";

  auto end = R"(
      </model>
    </sdf>)";

  std::string visual = R"(          <visual name='visual'>
            <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
                <label>10</label>
            </plugin>)";
  intro_string += pose;
  intro_string += visual;
  intro_string += geometry;
  intro_string += end;
  std::cout << intro_string;
  createEntityFromStr(intro_string, name);
}

std::string generate_capsule_str(int min, int max){
  std::string capsule = R"(
            <geometry> 
              <capsule>)";
  std::string radius = "<radius>" + random_num(min, max) + "</radius>";
  std::string length = "<length>" + random_num(min, max) + "</length>";
  capsule += radius;
  capsule += length;
  capsule += R"(</capsule>
            </geometry>
          </visual>
          <collision name='collision'>
            <geometry>                    
              <capsule>)";
    capsule += radius;
    capsule += length;
    capsule += R"(</capsule>
            </geometry>
          </collision>
        </link>)";
  return capsule;
}


std::string generate_box_str(int min, int max){
  std::string box = R"(
            <geometry>            <box>)";
  std::string size = "<size>" + random_num(min,max) +" " + random_num(min,max) + " " + random_num(min,max) +  "</size>";
  box += size;
  box += R"(</box></geometry>
          </visual>
          <collision name='collision'>
            <geometry>            <box>)";
  box += size;

  box += R"(</box></geometry>
          </collision>
        </link>)";
  return box;
}

std::string generate_sphere_str(int min, int max){
  std::string sphere = R"(
            <geometry>            <sphere>)";
  std::string radius = "<radius>" + random_num(min,max) + "</radius>";
  sphere += radius;
  sphere += R"(</sphere></geometry>
          </visual>
          <collision name='collision'>
            <geometry>            <sphere>)";
  sphere += radius;

  sphere += R"(</sphere></geometry>
          </collision>
        </link>)";
  return sphere;
}

std::string generate_pallet_str(int min, int max){
  std::string sphere = R"(
            <geometry>            <sphere>)";
  std::string radius = "<radius>" + random_num(min,max) + "</radius>";
  sphere += radius;
  sphere += R"(</sphere></geometry>
          </visual>
          <collision name='collision'>
            <geometry>            <sphere>)";
  sphere += radius;

  sphere += R"(</sphere></geometry>
          </collision>
        </link>)";
  return sphere;
}

void create_fuel_str(std::string name, std::string pose , std::string uri){
  removeEntityFromStr(name, ignition::msgs::Entity_Type_MODEL);
  removeEntityFromStr(name, ignition::msgs::Entity_Type_MODEL);
  removeEntityFromStr(name, ignition::msgs::Entity_Type_VISUAL);

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  std::string intro_string = R"(
    <?xml version="1.0" ?>
    <sdf version='1.7'>  <include>
    <static>true</static>)";
  
  intro_string += R"(
      <name>)" + name + R"(</name>)";

  intro_string += pose;


  std::string visual = R"(
            <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
                <label>10</label>
            </plugin>)";
  intro_string += visual;
  intro_string += uri;

    auto end = R"(
      </include>
    </sdf>)";
  intro_string += end;
  std::cout << intro_string;
  createEntityFromStr(intro_string, name);
}



  auto electrical_box = R"( 
        <?xml version="1.0" ?>
    <sdf version='1.7'>  <include>
      <static>true</static>
      <name>Electrical Box</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>https://fuel.ignitionrobotics.org/1.0/openrobotics/models/Electrical Box</uri>
    </include>
    </sdf>)";

//////////////////////////////////////////////////
int main(int argc, char **argv){

  const std::string str1 = R"(box)";
  const std::string str2 = R"(sphere)";
  const std::string str3 = R"(capsule)";
  const std::string str4 = R"(box2)";
  const std::string str5 = R"(sphere2)";
  const std::string str6 = R"(capsule2)";

  const std::string str7 = R"(box3)";
  const std::string str8 = R"(sphere3)";
  const std::string str9 = R"(capsule3)";
  
  const std::string str10 = R"(box4)";
  const std::string str11 = R"(sphere4)";
  const std::string str12 = R"(capsule4)";

  const std::string str13 = R"(box5)";
  const std::string str14 = R"(sphere5)";
  const std::string str15 = R"(capsule5)";

  std::list<int> pose = randomize_pose(true);
  create_random_entity(str1, deviate_poses(pose),  generate_box_str(1,10));
  create_random_entity(str2, deviate_poses(pose), generate_sphere_str(0,3));

  std::list<int> pose2 = randomize_pose(true);
  create_random_entity(str4, deviate_poses(pose2), generate_box_str(1,5));
  create_random_entity(str5, deviate_poses(pose2),generate_sphere_str(0,1));

  std::list<int> pose3 = randomize_pose(true);
  create_random_entity(str7, deviate_poses(pose3), generate_box_str(2,5));
  create_random_entity(str8, deviate_poses(pose3),generate_sphere_str(0,2));

  // std::list<int> pose4 = randomize_pose(true);
  // create_random_entity(str10, deviate_poses(pose4), generate_box_str(3,7));
  // create_random_entity(str11, deviate_poses(pose4),generate_sphere_str(0,1));

  // std::list<int> pose5 = randomize_pose(true);
  // create_random_entity(str13, deviate_poses(pose5), generate_box_str(1,2));
  // create_random_entity(str14, deviate_poses(pose5),generate_sphere_str(0,1));

  // create_list_of_poses();



  std::list<int> pose6 = randomize_pose_z(true);
  std::list<int> pose7 = randomize_pose_z(true);
  std::list<int> pose8 = randomize_pose_z(true);
  std::list<int> pose9 = randomize_pose_z(true);

  std::string cone = "<uri>https://fuel.ignitionrobotics.org/1.0/adlarkin/models/Construction Cone Label Test</uri>";
  std::string cone_str = "cone";
  std::string pallet = "<uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/pallet</uri>";
  std::string pallet_str = "pallet";
  std::string pallet_box = "<uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/pallet_box_mobile</uri>";
  std::string palletb_str = "pallet_box";
  std::string electrical_box = "<uri>https://fuel.ignitionrobotics.org/1.0/openrobotics/models/Electrical Box</uri>";
  std::string electrical_str = "electrical_box";


  create_fuel_str(cone_str, deviate_poses(pose6), cone);
  create_fuel_str(pallet_str, deviate_poses(pose7), pallet);
  create_fuel_str(palletb_str, deviate_poses(pose8), pallet_box);
  create_fuel_str(electrical_str, deviate_poses(pose9), electrical_box);


  }

//TODO:
// create list of objects to create from, either collection of spheres or from ignition fuel
// create random positions within 30,30,30 to -30, -30, -30
// remove old objects
// spawn the new ones<include>
// <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/TrolleyBox1</uri>

