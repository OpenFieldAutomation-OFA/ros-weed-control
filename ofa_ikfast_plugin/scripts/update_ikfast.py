#! /usr/bin/env python3

# This is an adapted version of 
# moveit2/moveit_kinematics/ikfast_kinematics_plugin/scripts/create_ikfast_moveit_plugin.py

import re
import os
from lxml import etree
import argparse

# Allowed search modes, see SEARCH_MODE enum in template file
search_modes = ["OPTIMIZE_MAX_JOINT", "OPTIMIZE_FREE_JOINT"]

robot_name = "ofa_robot"
ikfast_plugin_pkg = "ofa_ikfast_plugin"
base_link_name = "base_link"
eef_link_name = "eef_link"
planning_group_name = "arm"
search_mode = search_modes[0]

script_folder = os.path.dirname(__file__)
plugin_folder = script_folder + "/.."
template_folder = script_folder + "/../templates"

namespace = robot_name + "_" + planning_group_name
plugin_name = namespace + "/IKFastKinematicsPlugin"

def create_parser():
    parser = argparse.ArgumentParser(
        description="Generate an IKFast MoveIt kinematic plugin"
    )
    parser.add_argument(
        "ikfast_output_path",
        help="The full path to the analytic IK solution output by IKFast"
    )
    return parser


def update_deps(reqd_deps, req_type, e_parent):
    curr_deps = [e.text for e in e_parent.findall(req_type)]
    missing_deps = set(reqd_deps) - set(curr_deps)
    for dep in missing_deps:
        etree.SubElement(e_parent, req_type).text = dep


def update_ikfast_package(args):
    replacements = dict(
        _ROBOT_NAME_=robot_name,
        _GROUP_NAME_=planning_group_name,
        _SEARCH_MODE_=search_mode,
        _EEF_LINK_=eef_link_name,
        _BASE_LINK_=base_link_name,
        _PACKAGE_NAME_=ikfast_plugin_pkg,
        _NAMESPACE_=namespace,
    )

    # Copy ikfast header file
    copy_file(
        template_folder + "/ikfast.h",
        plugin_folder + "/include/ikfast.h",
        "ikfast header file",
    )
    # Create ikfast plugin template
    copy_file(
        template_folder
        + "/ikfast_moveit_plugin_template.cpp",
        plugin_folder
        + "/src/"
        + robot_name
        + "_"
        + planning_group_name
        + "_ikfast_moveit_plugin.cpp",
        "ikfast plugin file",
        replacements,
    )

    # Create plugin definition .xml file
    ik_library_name = namespace + "_moveit_ikfast_plugin"
    plugin_def = etree.Element("library", path=ik_library_name)
    cl = etree.SubElement(
        plugin_def,
        "class",
        name=plugin_name,
        type=namespace + "::IKFastKinematicsPlugin",
        base_class_type="kinematics::KinematicsBase",
    )
    desc = etree.SubElement(cl, "description")
    desc.text = (
        "IKFast plugin for closed-form kinematics of {robot} {group}".format(
            robot=robot_name,
            group=planning_group_name,
        )
    )

    # Write plugin definition to file
    plugin_file_name = ik_library_name + "_description.xml"
    plugin_file_path = plugin_folder + "/" + plugin_file_name
    etree.ElementTree(plugin_def).write(
        plugin_file_path, xml_declaration=True, pretty_print=True, encoding="UTF-8"
    )
    print("Created plugin definition at  '%s'" % plugin_file_path)

    # Create CMakeLists file
    replacements.update(dict(_LIBRARY_NAME_=ik_library_name))
    copy_file(
        template_folder + "/CMakeLists.txt",
        plugin_folder + "/CMakeLists.txt",
        "cmake file",
        replacements,
    )

    # Add plugin export to package manifest
    parser = etree.XMLParser(remove_blank_text=True)
    package_file_name = plugin_folder + "/package.xml"
    package_xml = etree.parse(package_file_name, parser).getroot()

    # Make sure at least all required dependencies are in the depends lists
    build_deps = [
        "liblapack-dev",
        "moveit_core",
        "pluginlib",
        "rclcpp",
        "tf2_kdl",
        "tf2_eigen",
    ]
    run_deps = ["liblapack-dev", "moveit_core", "pluginlib", "rclcpp"]

    update_deps(build_deps, "build_depend", package_xml)
    update_deps(run_deps, "exec_depend", package_xml)

    # Check that plugin definition file is in the export list
    new_export = etree.Element("moveit_core", plugin="${prefix}/" + plugin_file_name)

    export_element = package_xml.find("export")
    if export_element is None:
        export_element = etree.SubElement(package_xml, "export")

    found = False
    for el in export_element.findall("moveit_core"):
        found = etree.tostring(new_export) == etree.tostring(el)
        if found:
            break

    if not found:
        export_element.append(new_export)

    # Always write the package xml file, even if there are no changes, to ensure
    # proper encodings are used in the future (UTF-8)
    etree.ElementTree(package_xml).write(
        package_file_name, xml_declaration=True, pretty_print=True, encoding="UTF-8"
    )
    print("Wrote package.xml at  '%s'" % package_file_name)


def copy_file(src_path, dest_path, description, replacements=None):
    if not os.path.exists(src_path):
        raise Exception("Can't find %s at '%s'" % (description, src_path))

    if replacements is None:
        replacements = dict()

    with open(src_path, "r") as f:
        content = f.read()

    # replace templates
    for key, value in replacements.items():
        content = re.sub(key, value, content)

    with open(dest_path, "w") as f:
        f.write(content)
    print("Created %s at '%s'" % (description, dest_path))


def main():
    parser = create_parser()
    args = parser.parse_args()

    if not os.path.exists(args.ikfast_output_path):
        raise Exception("Can't find IKFast source code at " + args.ikfast_output_path)

    update_ikfast_package(args)


if __name__ == "__main__":
    main()
