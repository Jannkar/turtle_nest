#include "turtle_nest/package_generators/base_package_generator.h"
#include "turtle_nest/file_utils.h"
#include "turtle_nest/generate_launch.h"
#include "turtle_nest/generate_params.h"
#include "turtle_nest/node_generators/base_node_generator.h"
#include "turtle_nest/node_generators/node_generator_factory.h"
#include "turtle_nest/node_type_enum.h"
#include "turtle_nest/string_tools.h"

#include <QProcess>


BasePackageGenerator::BasePackageGenerator(PackageInfo pkg_info)
  : pkg_info(pkg_info)
{
  // ROS automatically populates the email from maintainer name, if email is not provided.
  // If the name contains invalid characters (or spaces) for email, the package building
  // will fail. This is why we need to set this default email instead of leaving it empty.
  if (pkg_info.maintainer_email.isEmpty()){
    pkg_info.maintainer_email = "todo@todo.todo";
  }

  if (pkg_info.license.isEmpty()){
    pkg_info.license = "TODO: License declaration";
  }
}


void BasePackageGenerator::create_package(){
  create_directory(pkg_info.workspace_path);
  create_package_impl();

  // Add watermark
  QFile file(QDir(pkg_info.package_path).filePath("package.xml"));
  if (file.open(QIODevice::Append | QIODevice::Text)) {
    QTextStream out(&file);
    out << "<!-- This package was generated using the Turtle Nest ROS 2 package creator -->\n";
    file.close();
  } else {
    qDebug() << "Failed to open the package.xml for appending watermark.";
  }
}

void BasePackageGenerator::create_launch_and_params(QString launch_name, QString params_file_name, QString node_name, bool composable_launch){
  // For now, we share the launch and params creation, as the CMakeLists or
  // setup.py are populated at once.
  bool create_launch = (launch_name != "");
  bool create_config = (params_file_name != "");

  // Generate launch file
  if (create_launch) {
    generate_launch_file(
        pkg_info.package_path, pkg_info.package_name, launch_name + ".py", params_file_name, node_type, node_name);
  }

  // Generate parameters file
  if (create_config) {
    generate_params_file(pkg_info.package_path, params_file_name + ".yaml", node_name, "");
  }

  add_launch_and_params_to_config_(pkg_info.package_path, create_launch, create_config);

}


void BasePackageGenerator::create_node(QString node_name, NodeType node_type, bool create_config){
  if (!node_name.isEmpty()) {
    std::unique_ptr<BaseNodeGenerator> node_generator = create_node_generator(pkg_info.package_type);
    NodeOptions node_options{
        node_name,
        node_type,
        create_config,   // add_params
    };
    node_generator->add_node(node_options, pkg_info.package_path, pkg_info.package_name);
  }
}


QStringList BasePackageGenerator::create_command(QString type) const
{
  QStringList command_list;

  command_list
      << "pkg" << "create"
      << "--build-type" << type
      << "--license" << pkg_info.license;

  if (!pkg_info.description.trimmed().isEmpty()) {
    command_list << "--description" << escape_xml(pkg_info.description);
  }
  if (!pkg_info.maintainer.trimmed().isEmpty()) {
    command_list << "--maintainer-name" << escape_xml(pkg_info.maintainer);
  }

  command_list << "--maintainer-email" << pkg_info.maintainer_email
               << pkg_info.package_name;
  return command_list;
}

void BasePackageGenerator::run_command(QStringList command) const
{
  QProcess builder;
  builder.setProcessChannelMode(QProcess::MergedChannels);
  builder.setWorkingDirectory(pkg_info.workspace_path);
  builder.start("ros2", command);
  qInfo().noquote() << "Running command: ros2" << builder.arguments().join(" ");

  if (!builder.waitForFinished()) {
    throw std::runtime_error("Failed to create a new package. Is ROS 2 installed?");
  }
  if (builder.exitCode() != 0) {
    auto error_message = builder.readAll();
    throw std::runtime_error("Running the command failed: " + error_message);
  }

  qInfo().noquote() << builder.readAll();
}
