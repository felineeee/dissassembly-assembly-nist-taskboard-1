#include "project_perception/scene_loader.hpp"

SceneLoader::SceneLoader(const rclcpp::NodeOptions& options) : Node("irb1200_scene_loader", options)
{
  this->declare_parameter<std::string>("scene_yaml_path", "");

  // Manually checks if the MoveGroup service is alive
  service_ = this->create_service<project_interfaces::srv::SceneLoad>(
      "load_planning_scene",
      [this](const std::shared_ptr<project_interfaces::srv::SceneLoad::Request> req,
             std::shared_ptr<project_interfaces::srv::SceneLoad::Response> res) { this->spawnScene(req, res); });

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
    this->timer_->cancel();

    // Debug
    // this->spawnDebugBox();
    // Check service inside the callback
    auto client = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!client->wait_for_service(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "MoveGroup not found! Scene loader aborting.");
      return;
    }

    std::string yaml_path = this->get_parameter("scene_yaml_path").as_string();
    if (!yaml_path.empty())
    {
      this->spawnScene(yaml_path);
    }
  });
}

std::string SceneLoader::resolvePath(const std::string& path)
{
  const std::string prefix = "project_description://";
  if (path.find(prefix) == 0)
  {
    std::string rel_path = path.substr(prefix.length());
    std::string share_dir = ament_index_cpp::get_package_share_directory("project_description");
    return share_dir + "/" + rel_path;
  }
  return path;
}

void SceneLoader::spawnDebugBox()
{
  moveit_msgs::msg::CollisionObject co;

  co.id = "debug_box";
  co.header.frame_id = "base_link";

  // 1. Define a 10cm x 10cm x 10cm cube
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = { 0.1, 0.1, 0.1 };

  // 2. Define the pose (1 meter in front of the robot)
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.05;  // Half the height so it sits ON the ground
  pose.orientation.w = 1.0;

  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(pose);
  co.operation = co.ADD;

  // 3. Apply the object
  if (this->psi_.applyCollisionObjects({ co }))
  {
    RCLCPP_INFO(this->get_logger(), "Successfully spawned debug box at [0.5, 0, 0.05]");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to spawn debug box.");
  }
}

bool SceneLoader::spawnScene(const std::string& yaml_path)
{
  YAML::Node config = YAML::LoadFile(yaml_path);
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  for (const auto& obj : config["objects"])
  {
    std::string object_name;

    moveit_msgs::msg::CollisionObject co;
    object_name = obj["id"].as<std::string>();
    co.id = object_name;
    co.header.frame_id = obj["frame_id"].as<std::string>();
    co.header.stamp = this->now();

    // Load Mesh
    std::string path = obj["mesh_path"].as<std::string>();
    std::string resolved_path = resolvePath(path);
    if (resolved_path.empty())
    {
      RCLCPP_ERROR(rclcpp::get_logger("scene_loader"), "Could not resolve path: %s", path.c_str());
      continue;
    }

    Eigen::Vector3d scale(0.001, 0.001, 0.001);
    shapes::Mesh* m = shapes::createMeshFromResource("file://" + resolved_path);  // DEBUG, remove scale param
    if (!m)
    {
      RCLCPP_ERROR(rclcpp::get_logger("irb1200_scene_loader"), "Failed to load mesh from: %s", resolved_path.c_str());
      continue;
    }

    // Diagnostic
    // Looking for mesh bound and determine its center
    // Out of suspect that stl file is not set at origin
    // double min_x = std::numeric_limits<double>::max();
    // double max_x = std::numeric_limits<double>::lowest();
    // double min_y = std::numeric_limits<double>::max();
    // double max_y = std::numeric_limits<double>::lowest();
    // double min_z = std::numeric_limits<double>::max();
    // double max_z = std::numeric_limits<double>::lowest();

    // for (unsigned int i = 0; i < m->vertex_count; ++i)
    // {
    //   min_x = std::min(min_x, (double)m->vertices[3 * i + 0]);
    //   max_x = std::max(max_x, (double)m->vertices[3 * i + 0]);
    //   min_y = std::min(min_y, (double)m->vertices[3 * i + 1]);
    //   max_y = std::max(max_y, (double)m->vertices[3 * i + 1]);
    //   min_z = std::min(min_z, (double)m->vertices[3 * i + 2]);
    //   max_z = std::max(max_z, (double)m->vertices[3 * i + 2]);
    // }

    // double center_x = (min_x + max_x) / 2.0;
    // double center_y = (min_y + max_y) / 2.0;
    // double center_z = (min_z + max_z) / 2.0;

    // RCLCPP_WARN(this->get_logger(), "MESH DIAGNOSTICS for %s:", object_name.c_str());
    // RCLCPP_WARN(this->get_logger(), "  Mesh bounds: X[%.4f, %.4f], Y[%.4f, %.4f], Z[%.4f, %.4f]", min_x, max_x,
    // min_y,
    //             max_y, min_z, max_z);
    // RCLCPP_WARN(this->get_logger(), "  Mesh center: [%.4f, %.4f, %.4f]", center_x, center_y, center_z);
    // RCLCPP_WARN(this->get_logger(), "  Mesh dimensions: [%.4f, %.4f, %.4f]", max_x - min_x, max_y - min_y,
    //             max_z - min_z);

    // for (unsigned int i = 0; i < m->vertex_count; ++i)
    // {
    //   m->vertices[3 * i + 0] -= center_x;
    //   m->vertices[3 * i + 1] -= center_y;
    //   m->vertices[3 * i + 2] -= center_z;
    // }

    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(m, shape_msg);
    co.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
    delete m;  // Prevent memory leak

    // Parse Pose [x, y, z, r, p, y]
    auto pose_data = obj["pose"].as<std::vector<double>>();
    geometry_msgs::msg::Pose p;
    p.position.x = pose_data[0];
    p.position.y = pose_data[1];
    p.position.z = pose_data[2];

    tf2::Quaternion q;
    q.setRPY(pose_data[3], pose_data[4], pose_data[5]);
    p.orientation = tf2::toMsg(q);

    RCLCPP_INFO(this->get_logger(), "--- Spawned object name: %s ---", object_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Input XYZ (m): [%.4f, %.4f, %.4f]", pose_data[0], pose_data[1], pose_data[2]);
    RCLCPP_INFO(this->get_logger(), "Input RPY (rad): [%.4f, %.4f, %.4f]", pose_data[3], pose_data[4], pose_data[5]);
    RCLCPP_INFO(this->get_logger(), "Output Quat (x,y,z,w): [%.4f, %.4f, %.4f, %.4f]", p.orientation.x, p.orientation.y,
                p.orientation.z, p.orientation.w);

    // co.mesh_poses.push_back(p);
    co.pose = p;
    co.operation = co.ADD;
    collision_objects.push_back(co);
  }

  // Adding object to the scene
  if (this->psi_.applyCollisionObjects(collision_objects))
  {
    RCLCPP_INFO(this->get_logger(), "Scene successfully applied to move_group.");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to apply scene. Is move_group running?");
    return false;
  }

  // Checking if its successfully spawned
  std::vector<std::string> object_ids;
  for (const auto& co : collision_objects)
    object_ids.push_back(co.id);

  auto known_objects = this->psi_.getObjects(object_ids);
  if (known_objects.size() == object_ids.size())
  {
    RCLCPP_INFO(rclcpp::get_logger("scene_loader"), "Confirmed: All objects spawned.");
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("scene_loader"), "Objects sent, but not yet detected in Planning Scene.");
  }

  auto names = this->psi_.getKnownObjectNames();
  if (names.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "CRITICAL: Planning Scene is still empty after apply!");
    return false;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Confirmed: %zu objects in Planning Scene.", names.size());
  }
  return true;
}
void SceneLoader::spawnScene(const std::shared_ptr<project_interfaces::srv::SceneLoad::Request> req,
                             std::shared_ptr<project_interfaces::srv::SceneLoad::Response> res)
{
  std::string path = req->scene_yaml_path;

  bool result = this->spawnScene(path);
  res->success = result;
  res->message = result ? "Scene loaded successfully" : "Failed to load scene";
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SceneLoader>());
  rclcpp::shutdown();
  return 0;
}
