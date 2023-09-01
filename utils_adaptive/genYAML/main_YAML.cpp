/* 
Generate two yaml files of inertial parameters (only for arm + hand) to use standard equation of dynamic and regressor.

    The inertial parameters are referred to joints' frame(!) of denavit parametrization.
    Denavit-Hartenberg parametrization table used is:
        DH_table << 0,      -M_PI_2,  0.3330, 0,
                    0,      M_PI_2,   0,      0,
                    0.0825, M_PI_2,   0.3160, 0,
                    -0.0825,-M_PI_2,  0,      0,
                    0,      M_PI_2,   0.384,  0,
                    0.088,  M_PI_2,   0,      0,
                    0,      0,        0.107,  0;

    For standard equation of dynamic the format of link's parameters are [m, CoM_x, CoMy, CoM_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz].
    For regressor the format of link's parameters are [m, m*CoM_x, m*CoMy, m*CoM_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz].

    Parameters manipulated are obtained with transformation of URDF file from:"
        - $(find franka_gazebo)/test/launch/panda-gazebo.urdf"
        - $(find franka_description)/robots/common/franka_robot.xacro"
        - $(find franka_description)/robots/common/inertial.yaml"
*/

#include <iostream>
#include <cmath>
#include <filesystem>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include "library/urdf2dh_inertial.h"

#define NUMLINKS 7 // 7 links + hand

using namespace regrob;

using std::cout;
using std::endl;

bool use_gripper = false;
bool copy_flag = true;

std::string path_yaml_DH = "../generatedFiles/inertial_DH.yaml";
std::string path_yaml_DH_REG = "../generatedFiles/inertial_DH_reg.yaml";
std::string path_yaml_DH_DYN = "../generatedFiles/inertial_DH_dyn.yaml";
//std::string path_copy_DH = "../../config/inertial_DH.yaml";
std::string path_copy_DH_REG = "../../config/inertial_DH_REG.yaml";
std::string path_copy_DH_DYN = "../../config/inertial_DH_DYN.yaml";

int main(){

    std::vector<LinkProp> links_prop;
    std::vector<LinkProp> links_prop_DH;
    std::vector<LinkProp> links_prop_DH2REG;
    std::vector<urdf2dh_T> transform;
    
    int numTransform;

    if (use_gripper){
        numTransform = NUMLINKS + 1;
    } else {
        numTransform = NUMLINKS;
    }

    links_prop_DH.resize(numTransform);
    links_prop_DH2REG.resize(NUMLINKS);

    transform.resize(numTransform);
    transform[0].rpy = {+M_PI_2,0,0};   // link transform
    transform[1].rpy = {-M_PI_2,0,0};
    transform[2].rpy = {-M_PI_2,0,0};
    transform[3].rpy = {+M_PI_2,0,0};
    transform[4].rpy = {-M_PI_2,0,0};
    transform[5].rpy = {-M_PI_2,0,0};
    
    transform[0].xyz = {0,0,0};         // link 1
    transform[1].xyz = {0,0,0};
    transform[2].xyz = {-0.0825,0,0};
    transform[3].xyz = {0.0825,0,0};
    transform[4].xyz = {0,0,0};
    transform[5].xyz = {-0.088,0,0};

    if (use_gripper){
        transform[6].rpy = {0,0,0+M_PI_4};
        transform[6].xyz = {0,0,-0.107-0.1034};
        transform[7].rpy = {0,0,0};         // hand
        transform[7].xyz = {0,0,0};         // hand
    } else {
        transform[6].rpy = {0,0,0};
        transform[6].xyz = {0,0,-0.107};
    }

    //------------------------------------Extract Variables-----------------------------------------//

    try {

        YAML::Node config = YAML::LoadFile("yaml/inertial.yaml");
        
        for (const auto& node : config) {

            LinkProp properties;
            std::string linkName = node.first.as<std::string>();
            YAML::Node mass = node.second["mass"];
            YAML::Node parI = node.second["inertia"];
            YAML::Node origin = node.second["origin"];
            std::string xyzStr, rpyStr;
            
            properties.name = linkName;
            properties.mass = mass.as<double>();
            properties.parI[0] = parI["xx"].as<double>();
            properties.parI[1] = parI["xy"].as<double>();
            properties.parI[2] = parI["xz"].as<double>();
            properties.parI[3] = parI["yy"].as<double>();
            properties.parI[4] = parI["yz"].as<double>();
            properties.parI[5] = parI["zz"].as<double>();
            
            if (origin["xyz"]) {
                xyzStr = origin["xyz"].as<std::string>();
                std::istringstream xyzStream(xyzStr);            
                xyzStream >> properties.xyz[0] >> properties.xyz[1] >> properties.xyz[2];
            } else {
                std::cerr << "Missing 'xyz' in the 'origin' node." << std::endl;
                continue;
            }

            if (origin["rpy"]) {
                rpyStr = origin["rpy"].as<std::string>();
                std::istringstream rpyStream(rpyStr);            
                rpyStream >> properties.rpy[0] >> properties.rpy[1] >> properties.rpy[2];
            } else {
                std::cerr << "Missing 'rpy' in the 'origin' node." << std::endl;
                continue;
            }

            links_prop.push_back(properties);
        }
        
    } catch (const YAML::Exception& e) {
        std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
    }

    //-----------------------Convert inertial parameters from URDF to DH----------------------------//

    for (int i = 1; i<=numTransform; i++){
        trasformBodyInertial(transform[i-1].xyz,transform[i-1].rpy,links_prop[i],links_prop_DH[i-1]);
    }

    /* Merge link 7 parameters and hand parameters */

    if (use_gripper){

        LinkProp link_7, link_hand, newlink_7;
        
        link_7 = links_prop_DH[6];
        link_hand = links_prop_DH[7];
        mergeBodyInertial(link_7,link_hand,newlink_7);
        newlink_7.name = link_7.name;
        links_prop_DH[6] = newlink_7;
    }
    
    //----------------!!! Generate DH inertial parameters YAML only for TEST !!!-----------------------//
    
    try {

        YAML::Emitter emitter;
        emitter.SetIndent(2);
        emitter.SetSeqFormat(YAML::Flow);
        emitter << YAML::Comment(
            "Inertial parameters referred to Denavit-Hartenberg parametrization to use standard equation of dynamic.\n"
            "Obtained with transformation of URDF files from:\n"
            "   - $(find franka_gazebo)/test/launch/panda-gazebo.urdf\n"
            "   - $(find franka_description)/robots/common/franka_robot.xacro\n"
            "   - $(find franka_description)/robots/common/inertial.yaml \n"
            "Considering Denavit-Hartenberg parametrization of table:\n"
            "DH_table << 0,      -M_PI_2,  0.3330, 0,\n"
            "            0,      M_PI_2,   0,      0,\n"
            "            0.0825, M_PI_2,   0.3160, 0,\n"
            "            -0.0825,-M_PI_2,  0,      0,\n"
            "            0,      M_PI_2,   0.384,  0,\n"
            "            0.088,  M_PI_2,   0,      0,\n"
            "            0,      0,        0.107,  0;"  );
        emitter << YAML::Newline;
        
        for (int i=0; i<7; i++) {
            LinkProp link = links_prop_DH[i];
                
            YAML::Node linkNode;

            linkNode["mass"] = link.mass;
            linkNode["CoM_x"] = link.xyz[0];
            linkNode["CoM_y"] = link.xyz[1];
            linkNode["CoM_z"] = link.xyz[2];
            linkNode["Ixx"] = link.parI[0];
            linkNode["Ixy"] = link.parI[1];
            linkNode["Ixz"] = link.parI[2];
            linkNode["Iyy"] = link.parI[3];
            linkNode["Iyz"] = link.parI[4];
            linkNode["Izz"] = link.parI[5];

            std::string nodeName = link.name;

            emitter << YAML::BeginMap;
            emitter << YAML::Key << nodeName;
            emitter << linkNode;
            emitter << YAML::EndMap << YAML::Newline;
        }
        std::ofstream fout(path_yaml_DH);
        fout << emitter.c_str();
        fout.close();

        std::cout << "Successfully generated YAML file of inertial parameter for DH parametrization, \n path: " << path_yaml_DH << std::endl;

    } catch (const YAML::Exception& e) {
        std::cerr << "Error while generating YAML: " << e.what() << std::endl;
    }
    
    //------------------Generate YAML of inertial parameters for standard dynamic---------------------//

    try {

        YAML::Emitter emitter;
        emitter.SetIndent(2);
        emitter.SetSeqFormat(YAML::Flow);
        emitter << YAML::Comment(
            "Inertial parameters referred to joints' frame(!) of DH parametrization to use regressor.\n"
            "Obtained with transformation of URDF file from:\n"
            "   - $(find franka_gazebo)/test/launch/panda-gazebo.urdf\n"
            "   - $(find franka_description)/robots/common/franka_robot.xacro\n"
            "   - $(find franka_description)/robots/common/inertial.yaml \n"
            "Considering Denavit-Hartenberg parametrization of table:\n"
            "DH_table << 0,      -M_PI_2,  0.3330, 0,\n"
            "            0,      M_PI_2,   0,      0,\n"
            "            0.0825, M_PI_2,   0.3160, 0,\n"
            "            -0.0825,-M_PI_2,  0,      0,\n"
            "            0,      M_PI_2,   0.384,  0,\n"
            "            0.088,  M_PI_2,   0,      0,\n"
            "            0,      0,        0.107,  0;"  );
        emitter << YAML::Newline;
        
        for (int i=0; i<NUMLINKS; i++) {
            LinkProp link = links_prop_DH[i];
                
            YAML::Node linkNode;

            linkNode["mass"] = link.mass;
            linkNode["CoM_x"] = link.xyz[0];
            linkNode["CoM_y"] = link.xyz[1];
            linkNode["CoM_z"] = link.xyz[2];
            linkNode["Ixx"] = link.parI[0];
            linkNode["Ixy"] = link.parI[1];
            linkNode["Ixz"] = link.parI[2];
            linkNode["Iyy"] = link.parI[3];
            linkNode["Iyz"] = link.parI[4];
            linkNode["Izz"] = link.parI[5];

            std::string nodeName = link.name + "_params";

            emitter << YAML::BeginMap;
            emitter << YAML::Key << nodeName;
            emitter << YAML::Anchor(link.name);
            emitter << linkNode;
            emitter << YAML::EndMap << YAML::Newline;
        }

        YAML::Node control;
        std::string contName;
        contName = "default_controller";

        emitter << YAML::BeginMap;
        emitter << YAML::Key << contName << YAML::Anchor("default");
        emitter << YAML::BeginMap;            

        for (int i=0; i<7; i++) {
            LinkProp link = links_prop_DH[i];
            emitter << YAML::Key << link.name+"_DYN" << YAML::Value << YAML::Alias(link.name);
        } 
        
        emitter << YAML::EndMap << YAML::EndMap <<YAML::Newline;

        /* Backstepping controller */

        emitter << YAML::BeginMap;
        emitter << YAML::Key << "backstepping_controller" << YAML::Value << YAML::Alias("default");
        emitter << YAML::EndMap <<YAML::Newline;

        std::ofstream fout(path_yaml_DH_DYN);
        fout << emitter.c_str();
        fout.close();

        std::cout << "Successfully generated YAML file of inertial parameters for regressor,\n path: " << path_yaml_DH_DYN << std::endl;

    } catch (const YAML::Exception& e) {
        std::cerr << "Error while generating YAML: " << e.what() << std::endl;
    }

    //------------------Generate YAML of inertial parameters for regressor---------------------//
    
    LinkProp tmp_link;
    Eigen::Matrix3d I0,IG;
    Eigen::Vector3d dOG;
    double m;
    for(int i=0;i<NUMLINKS;i++){

        tmp_link = links_prop_DH[i];

        m = links_prop_DH[i].mass;
        dOG << tmp_link.xyz[0], tmp_link.xyz[1], tmp_link.xyz[2];
        IG = createI(tmp_link.parI);
        I0 = IG + m * hat(dOG) * hat(dOG).transpose();
        dOG = m*dOG;

        links_prop_DH2REG[i].name = tmp_link.name;
        links_prop_DH2REG[i].mass = tmp_link.mass;
        links_prop_DH2REG[i].xyz = {dOG[0],dOG[1],dOG[2]};
        links_prop_DH2REG[i].parI[0] = I0(0,0);
        links_prop_DH2REG[i].parI[1] = I0(0,1);
        links_prop_DH2REG[i].parI[2] = I0(0,2);
        links_prop_DH2REG[i].parI[3] = I0(1,1);
        links_prop_DH2REG[i].parI[4] = I0(1,2);
        links_prop_DH2REG[i].parI[5] = I0(2,2);
    }
    
    try {

        YAML::Emitter emitter;
        emitter.SetIndent(2);
        emitter.SetSeqFormat(YAML::Flow);
        emitter << YAML::Comment(
            "Inertial parameters referred to joints' frame(!) of DH parametrization to use regressor.\n"
            "Obtained with transformation of URDF file from:\n"
            "   - $(find franka_gazebo)/test/launch/panda-gazebo.urdf\n"
            "   - $(find franka_description)/robots/common/franka_robot.xacro\n"
            "   - $(find franka_description)/robots/common/inertial.yaml \n"
            "Considering Denavit-Hartenberg parametrization of table:\n"
            "DH_table << 0,      -M_PI_2,  0.3330, 0,\n"
            "            0,      M_PI_2,   0,      0,\n"
            "            0.0825, M_PI_2,   0.3160, 0,\n"
            "            -0.0825,-M_PI_2,  0,      0,\n"
            "            0,      M_PI_2,   0.384,  0,\n"
            "            0.088,  M_PI_2,   0,      0,\n"
            "            0,      0,        0.107,  0;"  );
        emitter << YAML::Newline;
        
        for (int i=0; i<NUMLINKS; i++) {
            LinkProp link = links_prop_DH2REG[i];
                
            YAML::Node linkNode;

            linkNode["mass"] = link.mass;
            linkNode["m_CoM_x"] = link.xyz[0];
            linkNode["m_CoM_y"] = link.xyz[1];
            linkNode["m_CoM_z"] = link.xyz[2];
            linkNode["Ixx"] = link.parI[0];
            linkNode["Ixy"] = link.parI[1];
            linkNode["Ixz"] = link.parI[2];
            linkNode["Iyy"] = link.parI[3];
            linkNode["Iyz"] = link.parI[4];
            linkNode["Izz"] = link.parI[5];

            std::string nodeName = link.name + "_params";

            emitter << YAML::BeginMap;
            emitter << YAML::Key << nodeName;
            emitter << YAML::Anchor(link.name);
            emitter << linkNode;
            emitter << YAML::EndMap << YAML::Newline;
        }

        YAML::Node control;
        std::string contName;
        contName = "default_controller";

        emitter << YAML::BeginMap;
        emitter << YAML::Key << contName << YAML::Anchor("default");
        emitter << YAML::BeginMap;            

        for (int i=0; i<7; i++) {
            LinkProp link = links_prop_DH[i];
            emitter << YAML::Key << link.name+"_REG" << YAML::Value << YAML::Alias(link.name);
        } 
        
        emitter << YAML::EndMap << YAML::EndMap <<YAML::Newline;

        /* Backstepping controller */

        emitter << YAML::BeginMap;
        emitter << YAML::Key << "backstepping_controller" << YAML::Value << YAML::Alias("default");
        emitter << YAML::EndMap <<YAML::Newline;

        std::ofstream fout(path_yaml_DH_REG);
        fout << emitter.c_str();
        fout.close();

        std::cout << "Successfully generated YAML file of inertial parameters for regressor,\n path: " << path_yaml_DH_REG << std::endl;

    } catch (const YAML::Exception& e) {
        std::cerr << "Error while generating YAML: " << e.what() << std::endl;
    }

    if (copy_flag){
        std::string absolutePath;
        std::filesystem::path sourcePath;
        std::filesystem::path sourceDestPath;
        absolutePath = std::filesystem::current_path();
        /* sourcePath = absolutePath + "/" + path_yaml_DH;
        sourceDestPath = path_copy_DH;
        std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::update_existing); */
        sourcePath = absolutePath + "/" + path_yaml_DH_REG;
        sourceDestPath = path_copy_DH_REG;
        std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::update_existing);
        sourcePath = absolutePath + "/" + path_yaml_DH_DYN;
        sourceDestPath = path_copy_DH_DYN;
        std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::update_existing);
        std::cout<<"Files yaml copied"<<std::endl;
    }   

    return 0;
}